#include "rubi_gazebo_plugins/rubi_unified_policy_plugin.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rubi_control_core/controller.hpp"
#include "rubi_control_core/policy_contract.hpp"
#include "rubi_control_core/policy_runner.hpp"

namespace rubi_gazebo_plugins {
namespace {

namespace rc = rubi_control_core;
using Trigger = std_srvs::srv::Trigger;

template <typename Container>
bool all_finite(const Container& values) {
  return std::all_of(values.begin(), values.end(),
                     [](const auto value) { return std::isfinite(value); });
}

template <typename Container>
bool exact_zero(const Container& values) {
  return std::all_of(values.begin(), values.end(),
                     [](const auto value) { return value == 0; });
}

template <typename Container>
double maximum_absolute(const Container& values) {
  double maximum = 0.0;
  for (const auto value : values) {
    maximum = std::max(maximum, std::abs(static_cast<double>(value)));
  }
  return maximum;
}

std::string environment_or(const char* name, const std::string& fallback) {
  const char* value = std::getenv(name);
  return value && value[0] != '\0' ? value : fallback;
}

bool environment_flag(const char* name, bool fallback) {
  const auto value = environment_or(name, fallback ? "true" : "false");
  return value == "1" || value == "true" || value == "TRUE";
}

std::vector<double> parse_thresholds(const std::string& value) {
  std::vector<double> thresholds;
  std::istringstream stream(value);
  std::string token;
  while (std::getline(stream, token, ',')) {
    const double threshold = std::stod(token);
    if (!std::isfinite(threshold) || threshold <= 0.0) {
      throw std::runtime_error("diagnostic thresholds must be finite and positive");
    }
    thresholds.push_back(threshold);
  }
  if (thresholds.empty() || !std::is_sorted(thresholds.begin(), thresholds.end())) {
    throw std::runtime_error("diagnostic thresholds must be a sorted nonempty list");
  }
  return thresholds;
}

const char* mode_name(rc::ControllerMode mode) {
  switch (mode) {
    case rc::ControllerMode::kTorqueOff:
      return "torque_off";
    case rc::ControllerMode::kWalkReady:
      return "walk_ready";
    case rc::ControllerMode::kPolicyOn:
      return "policy";
    case rc::ControllerMode::kEmergencyStop:
      return "emergency_stop";
  }
  return "unknown";
}

const char* observation_name(std::size_t index) {
  static constexpr std::array<const char*, rc::kObservationDim> names = {
      "angular_velocity_x", "angular_velocity_y", "angular_velocity_z",
      "projected_gravity_x", "projected_gravity_y", "projected_gravity_z",
      "joint_position_relative_L_HR", "joint_position_relative_L_HP",
      "joint_position_relative_L_KN", "joint_position_relative_R_HR",
      "joint_position_relative_R_HP", "joint_position_relative_R_KN",
      "joint_velocity_L_HR", "joint_velocity_L_HP", "joint_velocity_L_KN",
      "joint_velocity_R_HR", "joint_velocity_R_HP", "joint_velocity_R_KN",
      "previous_action_L_HR", "previous_action_L_HP", "previous_action_L_KN",
      "previous_action_R_HR", "previous_action_R_HP", "previous_action_R_KN",
      "gait_phase_sin", "gait_phase_cos", "gait_frequency", "gait_offset",
      "gait_duration", "gait_swing_height"};
  return names.at(index);
}

template <typename Container>
std::pair<double, std::size_t> maximum_absolute_with_index(const Container& values) {
  double maximum = -1.0;
  std::size_t index = 0;
  for (std::size_t i = 0; i < values.size(); ++i) {
    const double value = static_cast<double>(values[i]);
    if (!std::isfinite(value)) {
      return {std::numeric_limits<double>::infinity(), i};
    }
    const double magnitude = std::abs(value);
    if (magnitude > maximum) {
      maximum = magnitude;
      index = i;
    }
  }
  return {maximum, index};
}

std::string csv_escape(const std::string& value) {
  std::string escaped = "\"";
  for (const char character : value) {
    escaped += character == '\"' ? "\"\"" : std::string(1, character);
  }
  return escaped + "\"";
}

}  // namespace

class RubiUnifiedPolicyPlugin::Impl {
 public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
    model_ = std::move(model);
    world_ = model_->GetWorld();
    if (!model_ || !world_) {
      throw std::runtime_error("Gazebo model/world pointer is null");
    }

    configure_ros_namespace(sdf);
    node_ = gazebo_ros::Node::Get(sdf);
    if (!node_) {
      throw std::runtime_error("gazebo_ros::Node creation failed");
    }

    const auto core_share = ament_index_cpp::get_package_share_directory("rubi_control_core");
    encoder_path_ = environment_or("RUBI_GAZEBO_ENCODER",
                                   core_share + "/models/rubi_plane_onnx/encoder.onnx");
    policy_path_ = environment_or("RUBI_GAZEBO_POLICY",
                                  core_share + "/models/rubi_plane_onnx/policy.onnx");
    self_test_ = environment_flag("RUBI_GAZEBO_SELF_TEST", false);
    command_timeout_ = std::stod(environment_or("RUBI_GAZEBO_COMMAND_TIMEOUT", "0.5"));
    policy_test_ticks_ =
        std::stoi(environment_or("RUBI_GAZEBO_POLICY_TEST_TICKS", "500"));
    if (command_timeout_ < 0.0) {
      throw std::runtime_error("command_timeout must be nonnegative");
    }
    if (policy_test_ticks_ < 500) {
      throw std::runtime_error("policy test duration must be at least 500 physics ticks");
    }
    if (std::abs(world_->Physics()->GetMaxStepSize() - 0.002) > 1.0e-12) {
      throw std::runtime_error("Gazebo max_step_size is not canonical 0.002 s");
    }

    map_joints();
    configure_diagnostics();
    configure_ros_interfaces();
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&Impl::OnUpdate, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(),
                "plugin_load=PASS max_step_size=0.002 controller_thread=world_update "
                "ros_callbacks=command_snapshot_only self_test=%s",
                self_test_ ? "true" : "false");
  }

  void configure_diagnostics() {
    diagnostics_enabled_ = environment_flag("RUBI_GAZEBO_DIAGNOSTICS_ENABLED", false);
    if (!diagnostics_enabled_) {
      return;
    }
    diagnostics_every_inference_ =
        environment_flag("RUBI_GAZEBO_DIAGNOSTICS_EVERY_INFERENCE", true);
    diagnostics_stop_on_threshold_ =
        environment_flag("RUBI_GAZEBO_DIAGNOSTICS_STOP_ON_THRESHOLD", false);
    diagnostic_post_window_ =
        std::stoi(environment_or("RUBI_GAZEBO_DIAGNOSTICS_POST_WINDOW", "5"));
    if (diagnostic_post_window_ < 0) {
      throw std::runtime_error("diagnostic post window must be nonnegative");
    }
    diagnostic_thresholds_ = parse_thresholds(environment_or(
        "RUBI_GAZEBO_DIAGNOSTICS_THRESHOLDS", "10,100,1000,1e6,1e12,1e24"));
    diagnostic_threshold_crossed_.assign(diagnostic_thresholds_.size(), false);
    diagnostics_path_ = environment_or("RUBI_GAZEBO_DIAGNOSTICS_CSV", "");
    if (diagnostics_path_.empty()) {
      throw std::runtime_error("diagnostics enabled without a CSV path");
    }
    diagnostics_stream_.open(diagnostics_path_, std::ios::out | std::ios::trunc);
    if (!diagnostics_stream_) {
      throw std::runtime_error("failed to open diagnostics CSV: " + diagnostics_path_);
    }
    diagnostics_stream_ << std::setprecision(17);
    auto* contact_manager = world_->Physics()->GetContactManager();
    if (contact_manager) {
      contact_manager->SetNeverDropContacts(true);
    }
    write_diagnostic_header();
    RCLCPP_INFO(node_->get_logger(),
                "diagnostics=ENABLED csv=%s every_inference=%s stop_on_threshold=%s "
                "post_window=%d thresholds=%s controller_feedback=false",
                diagnostics_path_.c_str(), diagnostics_every_inference_ ? "true" : "false",
                diagnostics_stop_on_threshold_ ? "true" : "false", diagnostic_post_window_,
                environment_or("RUBI_GAZEBO_DIAGNOSTICS_THRESHOLDS",
                               "10,100,1000,1e6,1e12,1e24").c_str());
  }

  void write_diagnostic_header() {
    diagnostics_stream_
        << "sim_time,physics_tick,inference_count,self_test_stage,controller_mode,"
           "inference_ran,faulted,safe_zero,fault_reason,update_dt,paused,reset_event,"
           "new_threshold,first_divergent_index,first_divergent_name,";
    for (std::size_t i = 0; i < rc::kObservationDim; ++i) {
      diagnostics_stream_ << "obs_" << i << '_' << observation_name(i) << ',';
    }
    diagnostics_stream_
        << "obs_angular_velocity_max,obs_projected_gravity_max,"
           "obs_joint_position_relative_max,obs_joint_velocity_max,"
           "obs_previous_action_max,obs_phase_max,obs_gait_parameter_max,"
           "history_max,history_max_frame,history_max_index,";
    for (std::size_t i = 0; i < rc::kEncoderInputDim; ++i) {
      diagnostics_stream_ << "encoder_input_" << i << ',';
    }
    diagnostics_stream_ << "latent_max,latent_max_index,";
    for (std::size_t i = 0; i < rc::kLatentDim; ++i) {
      diagnostics_stream_ << "latent_" << i << ',';
    }
    diagnostics_stream_ << "policy_input_max,policy_input_max_index,";
    for (std::size_t i = 0; i < rc::kPolicyInputDim; ++i) {
      diagnostics_stream_ << "policy_input_" << i << ',';
    }
    diagnostics_stream_ << "action_max,action_max_index,";
    for (std::size_t i = 0; i < rc::kActionDim; ++i) {
      diagnostics_stream_ << "action_" << i << ',';
    }
    for (std::size_t i = 0; i < rc::kJointCount; ++i) {
      diagnostics_stream_ << "joint_position_" << i << ',';
    }
    for (std::size_t i = 0; i < rc::kJointCount; ++i) {
      diagnostics_stream_ << "joint_velocity_" << i << ',';
    }
    for (std::size_t i = 0; i < rc::kJointCount; ++i) {
      diagnostics_stream_ << "measured_effort_" << i << ',';
    }
    for (std::size_t i = 0; i < rc::kJointCount; ++i) {
      diagnostics_stream_ << "applied_effort_" << i << ',';
    }
    diagnostics_stream_
        << "base_world_x,base_world_y,base_world_z,base_world_qx,base_world_qy,"
           "base_world_qz,base_world_qw,base_world_linear_vx,base_world_linear_vy,"
           "base_world_linear_vz,base_world_angular_vx,base_world_angular_vy,"
           "base_world_angular_vz,imu_qx,imu_qy,imu_qz,imu_qw,imu_quaternion_norm,"
           "imu_omega_x,imu_omega_y,imu_omega_z,contact_count,body_contact_count,"
           "foot_contact_count,support_contact_count,max_penetration,contact_pairs\n";
  }

  template <typename Container>
  double block_max(const Container& values, std::size_t begin, std::size_t end) const {
    double maximum = 0.0;
    for (std::size_t i = begin; i < end; ++i) {
      maximum = std::max(maximum, std::abs(static_cast<double>(values[i])));
    }
    return maximum;
  }

  struct ContactSummary {
    unsigned int count{0};
    unsigned int body_count{0};
    unsigned int foot_count{0};
    unsigned int support_count{0};
    double max_penetration{0.0};
    std::string pairs;
  };

  ContactSummary contact_summary() const {
    ContactSummary summary;
    auto* manager = world_->Physics()->GetContactManager();
    if (!manager) {
      return summary;
    }
    std::ostringstream pairs;
    const auto model_name = model_->GetScopedName();
    for (unsigned int i = 0; i < manager->GetContactCount(); ++i) {
      const auto* contact = manager->GetContact(i);
      if (!contact || !contact->collision1 || !contact->collision2) {
        continue;
      }
      const std::string first = contact->collision1->GetScopedName();
      const std::string second = contact->collision2->GetScopedName();
      if (first.find(model_name) == std::string::npos &&
          second.find(model_name) == std::string::npos) {
        continue;
      }
      ++summary.count;
      const std::string pair = first + "|" + second;
      summary.body_count += pair.find("::BODY::") != std::string::npos ? 1U : 0U;
      summary.foot_count += pair.find("TIP") != std::string::npos ? 1U : 0U;
      summary.support_count += pair.find("rubi_support_fixture") != std::string::npos ? 1U : 0U;
      for (int point = 0; point < contact->count; ++point) {
        summary.max_penetration =
            std::max(summary.max_penetration, contact->depths[point]);
      }
      if (pairs.tellp() > 0) {
        pairs << ';';
      }
      pairs << pair;
    }
    summary.pairs = pairs.str();
    return summary;
  }

  bool write_diagnostics(const rc::RobotState& state,
                         const rc::UserCommand& command,
                         const rc::ControllerOutput& output,
                         double update_dt) {
    if (!diagnostics_enabled_ || !controller_) {
      return false;
    }
    const bool inference_event = output.inference_ran || output.faulted;
    const bool periodic_sample = diagnostics_every_inference_ && adapter_tick_ % 5 == 0;
    if (!inference_event && !periodic_sample) {
      return false;
    }

    const auto& observation = controller_->last_observation();
    const auto& history = controller_->history();
    const auto& latent = controller_->last_latent();
    const auto& policy_input = controller_->last_policy_input();
    const auto& action = controller_->held_action();
    const auto [observation_max, observation_index] =
        maximum_absolute_with_index(observation);
    const auto [history_max, history_index] = maximum_absolute_with_index(history);
    const auto [latent_max, latent_index] = maximum_absolute_with_index(latent);
    const auto [policy_input_max, policy_input_index] =
        maximum_absolute_with_index(policy_input);
    const auto [action_max, action_index] = maximum_absolute_with_index(action);

    double new_threshold = 0.0;
    if (controller_->mode() == rc::ControllerMode::kPolicyOn && inference_event) {
      for (std::size_t i = 0; i < diagnostic_thresholds_.size(); ++i) {
        if (!diagnostic_threshold_crossed_[i] &&
            observation_max >= diagnostic_thresholds_[i]) {
          diagnostic_threshold_crossed_[i] = true;
          const double crossed_threshold = diagnostic_thresholds_[i];
          if (new_threshold == 0.0) {
            new_threshold = crossed_threshold;
          }
          RCLCPP_WARN(node_->get_logger(),
                      "diagnostic_threshold_crossed threshold=%.9g obs_index=%zu "
                      "obs_name=%s obs_value=%.9g sim_time=%.9f inference=%llu",
                      crossed_threshold, observation_index, observation_name(observation_index),
                      static_cast<double>(observation[observation_index]),
                      state.sim_time_sec,
                      static_cast<unsigned long long>(controller_->inference_count()));
          if (first_divergent_index_ < 0) {
            first_divergent_index_ = static_cast<int>(observation_index);
            first_divergent_name_ = observation_name(observation_index);
            first_threshold_ = new_threshold;
            first_threshold_sim_time_ = state.sim_time_sec;
            first_threshold_inference_ = controller_->inference_count();
            if (diagnostics_stop_on_threshold_) {
              diagnostic_post_remaining_ = diagnostic_post_window_;
            }
          }
        }
      }
    }

    const auto world_pose = model_->WorldPose();
    const auto world_linear_velocity = model_->WorldLinearVel();
    const auto world_angular_velocity = model_->WorldAngularVel();
    const auto contacts = contact_summary();
    const double quaternion_norm = std::sqrt(
        state.base_orientation_xyzw[0] * state.base_orientation_xyzw[0] +
        state.base_orientation_xyzw[1] * state.base_orientation_xyzw[1] +
        state.base_orientation_xyzw[2] * state.base_orientation_xyzw[2] +
        state.base_orientation_xyzw[3] * state.base_orientation_xyzw[3]);

    diagnostics_stream_ << state.sim_time_sec << ',' << adapter_tick_ << ','
                        << controller_->inference_count() << ',' << stage_ << ','
                        << mode_name(controller_->mode()) << ','
                        << (output.inference_ran ? 1 : 0) << ','
                        << (output.faulted ? 1 : 0) << ','
                        << (output.safe_zero ? 1 : 0) << ','
                        << csv_escape(output.fault_reason) << ',' << update_dt << ','
                        << (world_->IsPaused() ? 1 : 0) << ',' << (command.reset ? 1 : 0)
                        << ',' << new_threshold << ',' << first_divergent_index_ << ','
                        << csv_escape(first_divergent_name_) << ',';
    for (const auto value : observation) {
      diagnostics_stream_ << value << ',';
    }
    diagnostics_stream_ << block_max(observation, 0, 3) << ','
                        << block_max(observation, 3, 6) << ','
                        << block_max(observation, 6, 12) << ','
                        << block_max(observation, 12, 18) << ','
                        << block_max(observation, 18, 24) << ','
                        << block_max(observation, 24, 26) << ','
                        << block_max(observation, 26, 30) << ',' << history_max << ','
                        << history_index / rc::kObservationDim << ','
                        << history_index % rc::kObservationDim << ',';
    for (const auto value : history) {
      diagnostics_stream_ << value << ',';
    }
    diagnostics_stream_ << latent_max << ',' << latent_index << ',';
    for (const auto value : latent) {
      diagnostics_stream_ << value << ',';
    }
    diagnostics_stream_ << policy_input_max << ',' << policy_input_index << ',';
    for (const auto value : policy_input) {
      diagnostics_stream_ << value << ',';
    }
    diagnostics_stream_ << action_max << ',' << action_index << ',';
    for (const auto value : action) {
      diagnostics_stream_ << value << ',';
    }
    for (const auto value : state.joint_position) {
      diagnostics_stream_ << value << ',';
    }
    for (const auto value : state.joint_velocity) {
      diagnostics_stream_ << value << ',';
    }
    for (const auto value : state.measured_effort) {
      diagnostics_stream_ << value << ',';
    }
    for (const auto value : commanded_effort_) {
      diagnostics_stream_ << value << ',';
    }
    diagnostics_stream_
        << world_pose.Pos().X() << ',' << world_pose.Pos().Y() << ','
        << world_pose.Pos().Z() << ',' << world_pose.Rot().X() << ','
        << world_pose.Rot().Y() << ',' << world_pose.Rot().Z() << ','
        << world_pose.Rot().W() << ',' << world_linear_velocity.X() << ','
        << world_linear_velocity.Y() << ',' << world_linear_velocity.Z() << ','
        << world_angular_velocity.X() << ',' << world_angular_velocity.Y() << ','
        << world_angular_velocity.Z() << ',' << state.base_orientation_xyzw[0] << ','
        << state.base_orientation_xyzw[1] << ',' << state.base_orientation_xyzw[2] << ','
        << state.base_orientation_xyzw[3] << ',' << quaternion_norm << ','
        << state.base_angular_velocity[0] << ',' << state.base_angular_velocity[1] << ','
        << state.base_angular_velocity[2] << ',' << contacts.count << ','
        << contacts.body_count << ',' << contacts.foot_count << ','
        << contacts.support_count << ',' << contacts.max_penetration << ','
        << csv_escape(contacts.pairs) << '\n';
    diagnostics_stream_.flush();

    if (diagnostics_stop_on_threshold_ && first_divergent_index_ >= 0 &&
        inference_event) {
      if (new_threshold == first_threshold_) {
        return diagnostic_post_window_ == 0;
      }
      if (diagnostic_post_remaining_ > 0) {
        --diagnostic_post_remaining_;
      }
      return diagnostic_post_remaining_ == 0;
    }
    return false;
  }

 private:
  void configure_ros_namespace(const sdf::ElementPtr& sdf) {
    const auto requested = environment_or("RUBI_GAZEBO_NAMESPACE", "/rubi_gazebo");
    if (!sdf->HasElement("ros")) {
      sdf->AddElement("ros");
    }
    auto ros = sdf->GetElement("ros");
    if (!ros->HasElement("namespace")) {
      ros->AddElement("namespace");
    }
    ros->GetElement("namespace")->Set(requested);
  }

  void map_joints() {
    const auto& names = rc::canonical_joint_order();
    rc::validate_joint_order(std::vector<std::string>(names.begin(), names.end()));
    RCLCPP_INFO(node_->get_logger(),
                "controller_index joint scoped_joint dof local_axis effort_limit api_axis");
    for (std::size_t i = 0; i < rc::kJointCount; ++i) {
      joints_[i] = model_->GetJoint(names[i]);
      if (!joints_[i] || joints_[i]->DOF() != 1) {
        throw std::runtime_error("missing or non-single-DoF joint " + names[i]);
      }
      const auto axis = joints_[i]->LocalAxis(0);
      const ignition::math::Vector3d expected =
          (i == 0 || i == 3) ? ignition::math::Vector3d::UnitX
                             : ignition::math::Vector3d::UnitY;
      if (!axis.Equal(expected, 1.0e-12) ||
          std::abs(joints_[i]->GetEffortLimit(0) - 90.0) > 1.0e-12) {
        throw std::runtime_error("joint axis/effort contract mismatch for " + names[i]);
      }
      RCLCPP_INFO(node_->get_logger(), "%zu %s %s %u [%.0f,%.0f,%.0f] %.1f 0", i,
                  names[i].c_str(), joints_[i]->GetScopedName().c_str(),
                  joints_[i]->DOF(), axis.X(), axis.Y(), axis.Z(),
                  joints_[i]->GetEffortLimit(0));
    }
    RCLCPP_INFO(node_->get_logger(),
                "G2 PASS mapping=true single_dof=true effort_limit=90 "
                "axis_api=Position(0)/GetVelocity(0)/GetForce(0)/SetForce(0) "
                "correction=LEGACY_AXIS_1_BUG_CORRECTION");
  }

  bool resolve_imu() {
    if (imu_) {
      return true;
    }
    auto body = model_->GetLink("BODY");
    if (!body || body->GetSensorCount() == 0) {
      return false;
    }
    const auto sensor_name = body->GetSensorName(0);
    auto sensor = gazebo::sensors::SensorManager::Instance()->GetSensor(sensor_name);
    imu_ = std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(sensor);
    if (!imu_) {
      return false;
    }
    const auto parent = imu_->ParentName();
    if (parent.find(model_->GetScopedName() + "::BODY") == std::string::npos) {
      throw std::runtime_error("IMU parent is not scoped RUBI::BODY");
    }
    imu_->SetActive(true);
    RCLCPP_INFO(node_->get_logger(),
                "imu_mapping=PASS scoped_name=%s parent=%s active=%s frame=local",
                imu_->ScopedName().c_str(), parent.c_str(), imu_->IsActive() ? "true" : "false");
    return true;
  }

  void configure_ros_interfaces() {
    command_subscription_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr message) {
          std::lock_guard<std::mutex> lock(command_mutex_);
          command_.linear_x = message->linear.x;
          command_.linear_y = message->linear.y;
          command_.angular_z = message->angular.z;
          ++command_.command_sequence;
        });
    joint_state_publisher_ =
        node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    action_publisher_ =
        node_->create_publisher<std_msgs::msg::Float64MultiArray>("action", 10);
    status_publisher_ = node_->create_publisher<std_msgs::msg::String>("status", 10);
    walk_ready_service_ = mode_service("~/walk_ready", rc::ControllerMode::kWalkReady, false);
    policy_on_service_ = mode_service("~/policy_on", rc::ControllerMode::kPolicyOn, false);
    torque_off_service_ = mode_service("~/torque_off", rc::ControllerMode::kTorqueOff, false);
    emergency_service_ = mode_service("~/emergency_stop",
                                      rc::ControllerMode::kEmergencyStop, true);
    reset_service_ = node_->create_service<Trigger>(
        "~/reset", [this](const Trigger::Request::SharedPtr,
                           Trigger::Response::SharedPtr response) {
          std::lock_guard<std::mutex> lock(command_mutex_);
          command_.reset = true;
          command_.emergency_stop = false;
          ++command_.reset_sequence;
          response->success = true;
          response->message = "reset queued for next Gazebo update";
        });
  }

  rclcpp::Service<Trigger>::SharedPtr mode_service(
      const std::string& name, rc::ControllerMode mode, bool emergency) {
    return node_->create_service<Trigger>(
        name, [this, mode, emergency](const Trigger::Request::SharedPtr,
                                     Trigger::Response::SharedPtr response) {
          std::lock_guard<std::mutex> lock(command_mutex_);
          command_.requested_mode = mode;
          command_.emergency_stop = emergency;
          ++command_.mode_sequence;
          response->success = true;
          response->message = "mode queued for next Gazebo update";
        });
  }

  rc::UserCommand snapshot_command() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return command_;
  }

  rc::RobotState read_state(double sim_time) const {
    rc::RobotState state;
    state.sim_time_sec = sim_time;
    for (std::size_t i = 0; i < rc::kJointCount; ++i) {
      state.joint_position[i] = joints_[i]->Position(0);
      state.joint_velocity[i] = joints_[i]->GetVelocity(0);
      state.measured_effort[i] = joints_[i]->GetForce(0);
    }
    const auto orientation = imu_->Orientation();
    const auto angular_velocity = imu_->AngularVelocity(true);
    state.base_orientation_xyzw = {
        orientation.X(), orientation.Y(), orientation.Z(), orientation.W()};
    state.base_angular_velocity = {
        angular_velocity.X(), angular_velocity.Y(), angular_velocity.Z()};
    return state;
  }

  void write_effort(const rc::JointArray& effort) {
    commanded_effort_ = effort;
    for (std::size_t i = 0; i < rc::kJointCount; ++i) {
      joints_[i]->SetForce(0, effort[i]);
    }
  }

  void initialize_controller() {
    auto policy = std::make_shared<rc::PolicyRunner>(encoder_path_, policy_path_);
    rc::PolicyContract contract;
    contract.command_timeout = command_timeout_;
    controller_ = std::make_unique<rc::Controller>(policy, contract);
    RCLCPP_INFO(node_->get_logger(),
                "policy_load=PASS encoder=%s policy=%s startup_mode=torque_off",
                encoder_path_.c_str(), policy_path_.c_str());
  }

  void publish(const rc::RobotState& state, const rc::ControllerOutput& output) {
    sensor_msgs::msg::JointState joints;
    const auto nanoseconds = static_cast<std::int64_t>(state.sim_time_sec * 1.0e9);
    joints.header.stamp = rclcpp::Time(nanoseconds, RCL_ROS_TIME);
    const auto& names = rc::canonical_joint_order();
    joints.name.assign(names.begin(), names.end());
    joints.position.assign(state.joint_position.begin(), state.joint_position.end());
    joints.velocity.assign(state.joint_velocity.begin(), state.joint_velocity.end());
    joints.effort.assign(state.measured_effort.begin(), state.measured_effort.end());
    joint_state_publisher_->publish(joints);

    std_msgs::msg::Float64MultiArray action;
    action.data.assign(output.action.begin(), output.action.end());
    action_publisher_->publish(action);

    std_msgs::msg::String status;
    std::ostringstream stream;
    stream << "mode=" << static_cast<int>(output.mode)
           << " inference=" << (output.inference_ran ? 1 : 0)
           << " safe_zero=" << (output.safe_zero ? 1 : 0)
           << " fault=" << (output.faulted ? output.fault_reason : "none");
    status.data = stream.str();
    status_publisher_->publish(status);
  }

  void require(bool condition, const std::string& message) {
    if (!condition) {
      throw std::runtime_error("Gazebo self-test: " + message);
    }
  }

  void advance_self_test(rc::UserCommand& command,
                         const rc::RobotState& state,
                         const rc::ControllerOutput& output) {
    ++stage_tick_;
    if (stage_ == 4 && output.faulted) {
      require(exact_zero(commanded_effort_),
              "G4 nonfinite action did not produce same-update safe-zero");
      RCLCPP_ERROR(node_->get_logger(),
                   "G4 BLOCKED tick=%d inference=%llu reason=%s safe_zero=true "
                   "G5=NOT_RUN",
                   stage_tick_,
                   static_cast<unsigned long long>(controller_->inference_count()),
                   output.fault_reason.c_str());
      stage_ = 60;
      stage_tick_ = 0;
      test_command_.emergency_stop = false;
      test_command_.reset = true;
      ++test_command_.reset_sequence;
      return;
    }
    if (stage_ == 1 && stage_tick_ == 250) {
      require(exact_zero(commanded_effort_), "G1 torque-off effort is not exact zero");
      require(controller_->inference_count() == 0, "G1 inference ran while torque-off");
      require(all_finite(state.joint_position), "G1 joint state is nonfinite");
      RCLCPP_INFO(node_->get_logger(), "G1 PASS torque_off ticks=250 inference=0 six_SetForce0_zero=true");
      stage_ = 3;
      stage_tick_ = 0;
      test_command_.requested_mode = rc::ControllerMode::kWalkReady;
      ++test_command_.mode_sequence;
    } else if (stage_ == 3 && stage_tick_ == 300) {
      const auto expected = rc::PolicyContract{}.default_pose;
      for (std::size_t i = 0; i < rc::kJointCount; ++i) {
        require(std::abs(output.target_position[i] - expected[i]) < 1.0e-9,
                "G3 walk-ready target mismatch");
        require(std::isfinite(output.effort[i]) && std::abs(output.effort[i]) <= 90.0,
                "G3 effort invalid");
      }
      RCLCPP_INFO(node_->get_logger(), "G3 PASS walk_ready duration=0.6 target_default=true effort_clamped=true");
      stage_ = 4;
      stage_tick_ = 0;
      inference_stage_start_ = controller_->inference_count();
      test_command_.requested_mode = rc::ControllerMode::kPolicyOn;
      ++test_command_.mode_sequence;
    } else if (stage_ == 4 && stage_tick_ == policy_test_ticks_) {
      const auto inference_delta = controller_->inference_count() - inference_stage_start_;
      RCLCPP_INFO(node_->get_logger(),
                  "G4 timing_probe ticks=%d inference_start=%llu inference_end=%llu delta=%llu "
                  "mode=%d faulted=%s fault_reason=%s",
                  policy_test_ticks_,
                  static_cast<unsigned long long>(inference_stage_start_),
                  static_cast<unsigned long long>(controller_->inference_count()),
                  static_cast<unsigned long long>(inference_delta),
                  static_cast<int>(controller_->mode()), output.faulted ? "true" : "false",
                  output.fault_reason.empty() ? "none" : output.fault_reason.c_str());
      require(inference_delta == static_cast<std::uint64_t>(policy_test_ticks_ / 5),
              "G4 inference decimation mismatch: actual=" +
                  std::to_string(inference_delta));
      require(all_finite(output.action) && all_finite(output.effort),
              "G4 nonfinite action/effort");
      require(controller_->last_observation().size() == 30 &&
                  controller_->history().size() == 300 &&
                  controller_->last_latent().size() == 3 &&
                  controller_->last_policy_input().size() == 36,
              "G4 dimensions mismatch");
      RCLCPP_INFO(node_->get_logger(),
                  "G4 PASS policy_zero_command ticks=%d inference=%llu "
                  "dims=30/300/3/36/6 imu_finite=true",
                  policy_test_ticks_, static_cast<unsigned long long>(inference_delta));
      stage_ = 5;
      stage_tick_ = 0;
      test_command_.linear_x = 0.1;
      ++test_command_.command_sequence;
    } else if (stage_ == 5) {
      if (stage_tick_ % 50 == 0) {
        ++test_command_.command_sequence;
      }
      if (stage_tick_ == 500) {
        require(std::abs(controller_->last_policy_input()[33] - 0.4F) < 1.0e-6,
                "G5 command scale mismatch");
        RCLCPP_INFO(node_->get_logger(), "G5 PASS cmd_x=0.1 rate=10Hz scaled=0.4 finite=true");
        stage_ = 6;
        stage_tick_ = 0;
        test_command_.linear_x = 0.0;
        ++test_command_.command_sequence;
      }
    } else if (stage_ == 6 && stage_tick_ == 5) {
      require(!exact_zero(commanded_effort_), "G6 pre-E-stop effort is zero");
      pre_estop_nonzero_ = true;
      test_command_.emergency_stop = true;
      estop_apply_next_tick_ = true;
    } else if (stage_ == 6 && estop_apply_next_tick_) {
      require(pre_estop_nonzero_ && exact_zero(commanded_effort_) && exact_zero(output.effort),
              "G6 same-tick E-stop SetForce axis0 zero failed");
      RCLCPP_INFO(node_->get_logger(),
                  "G6 PASS pre_effort_nonzero=true same_update_six_SetForce_axis0_zero=true");
      stage_ = 7;
      stage_tick_ = 0;
      estop_apply_next_tick_ = false;
      test_command_.emergency_stop = false;
      test_command_.reset = true;
      ++test_command_.reset_sequence;
    } else if (stage_ == 7 && stage_tick_ == 1) {
      require(exact_zero(controller_->history()) && exact_zero(controller_->held_action()) &&
                  controller_->gait_phase() == 0.0 && exact_zero(commanded_effort_),
              "G7 reset state mismatch");
      require(controller_->mode() == rc::ControllerMode::kTorqueOff,
              "G7 reset mode mismatch");
      RCLCPP_INFO(node_->get_logger(),
                  "G7 PASS model_reset=true history_zero=true action_zero=true phase=0 mode=torque_off");
      RCLCPP_INFO(node_->get_logger(),
                  "GAZEBO_CONTROLLER_SELF_TEST=PASS api_axis=0 clock_owner=gazebo_ros "
                  "stable_locomotion=NOT_TESTED");
      self_test_complete_ = true;
      world_->SetPaused(true);
    } else if (stage_ == 60 && stage_tick_ == 1) {
      require(exact_zero(controller_->history()) && exact_zero(controller_->held_action()) &&
                  controller_->gait_phase() == 0.0 && exact_zero(commanded_effort_),
              "post-G4 safety reset failed");
      test_command_.requested_mode = rc::ControllerMode::kPolicyOn;
      ++test_command_.mode_sequence;
      stage_ = 61;
      stage_tick_ = 0;
    } else if (stage_ == 61 && stage_tick_ == 5) {
      require(!exact_zero(commanded_effort_),
              "independent G6 pre-E-stop effort is zero");
      test_command_.emergency_stop = true;
      stage_ = 62;
      stage_tick_ = 0;
    } else if (stage_ == 62 && stage_tick_ == 1) {
      require(exact_zero(commanded_effort_) && exact_zero(output.effort),
              "independent G6 E-stop axis0 zero failed");
      RCLCPP_INFO(node_->get_logger(),
                  "G6 PASS independent_safety_sequence pre_effort_nonzero=true "
                  "same_update_six_SetForce_axis0_zero=true");
      test_command_.emergency_stop = false;
      test_command_.reset = true;
      ++test_command_.reset_sequence;
      stage_ = 63;
      stage_tick_ = 0;
    } else if (stage_ == 63 && stage_tick_ == 1) {
      require(exact_zero(controller_->history()) && exact_zero(controller_->held_action()) &&
                  controller_->gait_phase() == 0.0 && exact_zero(commanded_effort_),
              "independent G7 reset state mismatch");
      require(controller_->mode() == rc::ControllerMode::kTorqueOff,
              "independent G7 reset mode mismatch");
      RCLCPP_INFO(node_->get_logger(),
                  "G7 PASS after_blocked_G4 model_reset=true history_zero=true "
                  "action_zero=true phase=0 mode=torque_off");
      RCLCPP_ERROR(node_->get_logger(),
                   "GAZEBO_CONTROLLER_SELF_TEST=BLOCKED "
                   "reason=G4_nonfinite_network_output_before_1s G5=NOT_RUN "
                   "safety_safe_zero_G6_G7=PASS");
      self_test_complete_ = true;
      world_->SetPaused(true);
    }
  }

  void OnUpdate(const gazebo::common::UpdateInfo& info) {
    try {
      if (self_test_complete_) {
        return;
      }
      if (!resolve_imu()) {
        rc::JointArray zero{};
        write_effort(zero);
        return;
      }
      if (!controller_) {
        initialize_controller();
      }

      const double sim_time = info.simTime.Double();
      const double update_dt = last_sim_time_ < 0.0 ? 0.0 : sim_time - last_sim_time_;
      if (last_sim_time_ >= 0.0 && sim_time + 1.0e-12 < last_sim_time_) {
        controller_->reset();
        rc::JointArray zero{};
        write_effort(zero);
        RCLCPP_WARN(node_->get_logger(), "backward_time_jump core_reset=true safe_zero=true");
      }
      last_sim_time_ = sim_time;
      ++adapter_tick_;

      rc::UserCommand command = self_test_ ? test_command_ : snapshot_command();
      if (command.reset) {
        model_->Reset();
      }
      const auto state = read_state(sim_time);
      auto output = controller_->update(state, command);
      write_effort(output.effort);
      if (write_diagnostics(state, command, output, update_dt)) {
        rc::JointArray zero{};
        write_effort(zero);
        RCLCPP_WARN(node_->get_logger(),
                    "DIAGNOSTIC_STOP first_index=%d first_name=%s threshold=%.9g "
                    "sim_time=%.9f inference=%llu post_window=%d safe_zero=true",
                    first_divergent_index_, first_divergent_name_.c_str(), first_threshold_,
                    first_threshold_sim_time_,
                    static_cast<unsigned long long>(first_threshold_inference_),
                    diagnostic_post_window_);
        self_test_complete_ = true;
        world_->SetPaused(true);
        return;
      }
      if (output.faulted && !fault_logged_) {
        fault_logged_ = true;
        RCLCPP_ERROR(
            node_->get_logger(),
            "first_core_fault sim_time=%.9f stage=%d stage_tick=%d inference=%llu "
            "reason=%s history_finite=%s history_max_abs=%.9g "
            "q=[%.9g,%.9g,%.9g,%.9g,%.9g,%.9g] "
            "dq=[%.9g,%.9g,%.9g,%.9g,%.9g,%.9g] "
            "quat_xyzw=[%.9g,%.9g,%.9g,%.9g] omega=[%.9g,%.9g,%.9g]",
            sim_time, stage_, stage_tick_,
            static_cast<unsigned long long>(controller_->inference_count()),
            output.fault_reason.c_str(), all_finite(controller_->history()) ? "true" : "false",
            maximum_absolute(controller_->history()), state.joint_position[0], state.joint_position[1],
            state.joint_position[2], state.joint_position[3], state.joint_position[4],
            state.joint_position[5], state.joint_velocity[0], state.joint_velocity[1],
            state.joint_velocity[2], state.joint_velocity[3], state.joint_velocity[4],
            state.joint_velocity[5], state.base_orientation_xyzw[0],
            state.base_orientation_xyzw[1], state.base_orientation_xyzw[2],
            state.base_orientation_xyzw[3], state.base_angular_velocity[0],
            state.base_angular_velocity[1], state.base_angular_velocity[2]);
      }
      if (controller_->physics_tick_count() % 5 == 0) {
        publish(state, output);
      }
      if (self_test_) {
        const bool reset_consumed = command.reset;
        const auto consumed_reset_sequence = command.reset_sequence;
        advance_self_test(command, state, output);
        if (reset_consumed && test_command_.reset_sequence == consumed_reset_sequence) {
          test_command_.reset = false;
        }
      } else if (command.reset) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        if (command_.reset_sequence == command.reset_sequence) {
          command_.reset = false;
        }
      }
    } catch (const std::exception& error) {
      rc::JointArray zero{};
      write_effort(zero);
      RCLCPP_FATAL(node_->get_logger(), "GAZEBO_CONTROLLER_RUNTIME=FAIL reason=%s",
                   error.what());
      self_test_complete_ = true;
      world_->SetPaused(true);
    }
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::event::ConnectionPtr update_connection_;
  std::array<gazebo::physics::JointPtr, rc::kJointCount> joints_{};
  gazebo::sensors::ImuSensorPtr imu_;
  std::unique_ptr<rc::Controller> controller_;
  std::string encoder_path_;
  std::string policy_path_;
  bool self_test_{false};
  bool self_test_complete_{false};
  double command_timeout_{0.5};
  int policy_test_ticks_{500};
  double last_sim_time_{-1.0};
  std::mutex command_mutex_;
  rc::UserCommand command_;
  rc::UserCommand test_command_;
  rc::JointArray commanded_effort_{};
  int stage_{1};
  int stage_tick_{0};
  std::uint64_t inference_stage_start_{0};
  bool pre_estop_nonzero_{false};
  bool estop_apply_next_tick_{false};
  bool fault_logged_{false};
  bool diagnostics_enabled_{false};
  bool diagnostics_every_inference_{true};
  bool diagnostics_stop_on_threshold_{false};
  std::string diagnostics_path_;
  std::ofstream diagnostics_stream_;
  std::vector<double> diagnostic_thresholds_;
  std::vector<bool> diagnostic_threshold_crossed_;
  int diagnostic_post_window_{5};
  int diagnostic_post_remaining_{0};
  int first_divergent_index_{-1};
  std::string first_divergent_name_;
  double first_threshold_{0.0};
  double first_threshold_sim_time_{0.0};
  std::uint64_t first_threshold_inference_{0};
  std::uint64_t adapter_tick_{0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr action_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Service<Trigger>::SharedPtr walk_ready_service_;
  rclcpp::Service<Trigger>::SharedPtr policy_on_service_;
  rclcpp::Service<Trigger>::SharedPtr torque_off_service_;
  rclcpp::Service<Trigger>::SharedPtr emergency_service_;
  rclcpp::Service<Trigger>::SharedPtr reset_service_;
};

RubiUnifiedPolicyPlugin::RubiUnifiedPolicyPlugin() : impl_(std::make_unique<Impl>()) {}
RubiUnifiedPolicyPlugin::~RubiUnifiedPolicyPlugin() = default;

void RubiUnifiedPolicyPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
  try {
    impl_->Load(std::move(model), std::move(sdf));
  } catch (const std::exception& error) {
    gzerr << "RUBI unified policy plugin load failed: " << error.what() << '\n';
    throw;
  }
}

GZ_REGISTER_MODEL_PLUGIN(RubiUnifiedPolicyPlugin)

}  // namespace rubi_gazebo_plugins
