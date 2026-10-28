// GPU-rendered Livox (MID-360) LiDAR for Gazebo Classic 11.
//
// The stock ros2_livox plugin casts every Livox ray through ODE on the CPU.
// This plugin instead renders a dense range image with a `gpu_ray` sensor and
// samples it along the Livox scan pattern from the same CSV file, so the
// output keeps the non-repetitive MID-360 pattern while the ray tracing runs
// on the GPU.
//
// Output is compatible with ros2_livox: a livox_ros_driver2/CustomMsg on
// <topic> and a sensor_msgs/PointCloud2 (x, y, z) on <topic>_PointCloud2, both
// in the sensor frame. Like ros2_livox, out-of-range returns are published as
// points at the origin unless <drop_invalid> is true.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/GpuRaySensor.hh>
#include <gazebo_ros/node.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace gpu_livox_lidar
{

struct ScanDirection
{
  float x, y, z;      // unit vector in the sensor frame
  float azimuth;      // rad, wrapped to [-pi, pi)
  float elevation;    // rad
};

class GpuLivoxPlugin : public gazebo::SensorPlugin
{
public:
  void Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf) override
  {
    sensor_ = std::dynamic_pointer_cast<gazebo::sensors::GpuRaySensor>(sensor);
    if (!sensor_) {
      gzerr << "[gpu_livox] must be attached to a sensor of type gpu_ray\n";
      return;
    }
    node_ = gazebo_ros::Node::Get(sdf);
    auto logger = node_->get_logger();

    const std::string csv = Param<std::string>(sdf, "csv_file_name", "");
    samples_ = std::max(1, Param<int>(sdf, "samples", 20000));
    downsample_ = std::max(1, Param<int>(sdf, "downsample", 1));
    drop_invalid_ = Param<bool>(sdf, "drop_invalid", false);
    frame_id_ = Param<std::string>(sdf, "frame_id", sensor_->Name());
    const std::string topic = Param<std::string>(sdf, "topic", "/livox/lidar");

    if (!LoadPattern(csv)) {
      RCLCPP_ERROR(logger, "[gpu_livox] cannot read scan pattern '%s'", csv.c_str());
      return;
    }
    CheckCoverage();

    custom_pub_ = node_->create_publisher<livox_ros_driver2::msg::CustomMsg>(topic, 10);
    cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      topic + "_PointCloud2", 10);
    connection_ = sensor_->ConnectNewLaserFrame(
      std::bind(&GpuLivoxPlugin::OnFrame, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3, std::placeholders::_4,
      std::placeholders::_5));
    sensor_->SetActive(true);

    RCLCPP_INFO(
      logger, "[gpu_livox] %s: pattern %zu dirs, %d samples/scan (downsample %d), "
      "grid %dx%d, h [%.2f, %.2f] v [%.2f, %.2f] deg, range [%.2f, %.2f] m -> %s",
      sensor_->Name().c_str(), pattern_.size(), samples_, downsample_,
      sensor_->RangeCount(), sensor_->VerticalRangeCount(),
      sensor_->AngleMin().Degree(), sensor_->AngleMax().Degree(),
      sensor_->VerticalAngleMin().Degree(), sensor_->VerticalAngleMax().Degree(),
      sensor_->RangeMin(), sensor_->RangeMax(), topic.c_str());
  }

private:
  template<typename T>
  static T Param(const sdf::ElementPtr & sdf, const std::string & name, const T & fallback)
  {
    return sdf->HasElement(name) ? sdf->Get<T>(name) : fallback;
  }

  // CSV columns: index, azimuth [deg], zenith [deg] (ros2_livox convention:
  // the ray is Euler(0, zenith - 90deg, azimuth) applied to +x).
  bool LoadPattern(const std::string & path)
  {
    std::ifstream in(path);
    if (!in) {
      return false;
    }
    std::string line;
    std::getline(in, line);  // header
    const double d2r = M_PI / 180.0;
    while (std::getline(in, line)) {
      std::stringstream ss(line);
      std::string a, b, c;
      if (!std::getline(ss, a, ',') || !std::getline(ss, b, ',') || !std::getline(ss, c, ',')) {
        continue;
      }
      const double az = std::stod(b) * d2r;
      const double pitch = std::stod(c) * d2r - M_PI_2;
      // Rz(az) * Ry(pitch) * (1, 0, 0)
      ScanDirection dir;
      dir.x = static_cast<float>(std::cos(az) * std::cos(pitch));
      dir.y = static_cast<float>(std::sin(az) * std::cos(pitch));
      dir.z = static_cast<float>(-std::sin(pitch));
      dir.azimuth = static_cast<float>(std::atan2(dir.y, dir.x));
      dir.elevation = static_cast<float>(-pitch);
      pattern_.push_back(dir);
    }
    return !pattern_.empty();
  }

  void CheckCoverage()
  {
    float lo = pattern_.front().elevation, hi = lo;
    for (const auto & d : pattern_) {
      lo = std::min(lo, d.elevation);
      hi = std::max(hi, d.elevation);
    }
    const double vmin = sensor_->VerticalAngleMin().Radian();
    const double vmax = sensor_->VerticalAngleMax().Radian();
    if (lo < vmin - 1e-3 || hi > vmax + 1e-3) {
      RCLCPP_WARN(
        node_->get_logger(), "[gpu_livox] pattern elevation [%.2f, %.2f] deg exceeds the "
        "gpu_ray vertical range [%.2f, %.2f] deg; those rays use the nearest edge row",
        lo * 180 / M_PI, hi * 180 / M_PI, vmin * 180 / M_PI, vmax * 180 / M_PI);
    }
    const double hfov = sensor_->AngleMax().Radian() - sensor_->AngleMin().Radian();
    if (hfov < 2 * M_PI - 2 * (2 * M_PI / sensor_->RangeCount()) - 1e-6) {
      RCLCPP_WARN(node_->get_logger(), "[gpu_livox] horizontal FOV is not 360 deg");
    }
  }

  void OnFrame(
    const float * image, unsigned int width, unsigned int height, unsigned int depth,
    const std::string &)
  {
    if (!image || width < 2 || height < 2 || pattern_.empty()) {
      return;
    }
    const double hmin = sensor_->AngleMin().Radian();
    const double hmax = sensor_->AngleMax().Radian();
    const double vmin = sensor_->VerticalAngleMin().Radian();
    const double vmax = sensor_->VerticalAngleMax().Radian();
    const double hstep = (hmax - hmin) / (width - 1);
    const double vstep = (vmax - vmin) / (height - 1);
    const double rmin = sensor_->RangeMin();
    const double rmax = sensor_->RangeMax();
    const bool full_circle = (hmax - hmin + hstep) >= 2 * M_PI - 1e-6;

    const auto sim_time = sensor_->LastMeasurementTime();
    const rclcpp::Time stamp(sim_time.sec, sim_time.nsec, RCL_ROS_TIME);
    const double scan_period = sensor_->UpdateRate() > 0 ? 1.0 / sensor_->UpdateRate() : 0.1;

    livox_ros_driver2::msg::CustomMsg custom;
    custom.header.stamp = stamp;
    custom.header.frame_id = frame_id_;
    custom.timebase = static_cast<uint64_t>(stamp.nanoseconds());
    custom.points.reserve(samples_ / downsample_ + 1);
    std::vector<float> xyz;
    xyz.reserve(3 * (samples_ / downsample_ + 1));

    const size_t n = pattern_.size();
    for (int k = 0; k < samples_; k += downsample_) {
      const ScanDirection & d = pattern_[(start_ + k) % n];
      double fi = (d.azimuth - hmin) / hstep;
      if (full_circle) {
        fi = std::fmod(fi, static_cast<double>(width));
        if (fi < 0) {
          fi += width;
        }
      }
      long i = std::lround(fi);
      if (full_circle) {
        i %= static_cast<long>(width);
      }
      i = std::clamp<long>(i, 0, width - 1);
      const long j = std::clamp<long>(std::lround((d.elevation - vmin) / vstep), 0, height - 1);
      const size_t idx = (static_cast<size_t>(j) * width + static_cast<size_t>(i)) * depth;
      double range = image[idx];
      const double retro = depth > 1 ? image[idx + 1] : 0.0;
      const bool valid = std::isfinite(range) && range > rmin && range < rmax;
      if (!valid) {
        if (drop_invalid_) {
          continue;
        }
        range = 0.0;
      }
      livox_ros_driver2::msg::CustomPoint p;
      p.x = static_cast<float>(range * d.x);
      p.y = static_cast<float>(range * d.y);
      p.z = static_cast<float>(range * d.z);
      p.reflectivity = static_cast<uint8_t>(std::clamp(retro, 0.0, 255.0));
      p.offset_time = static_cast<uint32_t>(1e9 * scan_period * k / samples_);
      custom.points.push_back(p);
      xyz.insert(xyz.end(), {p.x, p.y, p.z});
    }
    start_ = (start_ + samples_) % n;
    custom.point_num = static_cast<uint32_t>(custom.points.size());
    custom_pub_->publish(custom);

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = custom.header;
    cloud.height = 1;
    cloud.width = custom.point_num;
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = 12;
    cloud.row_step = cloud.point_step * cloud.width;
    for (const char * name : {"x", "y", "z"}) {
      sensor_msgs::msg::PointField field;
      field.name = name;
      field.offset = static_cast<uint32_t>(4 * cloud.fields.size());
      field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field.count = 1;
      cloud.fields.push_back(field);
    }
    cloud.data.resize(xyz.size() * sizeof(float));
    std::memcpy(cloud.data.data(), xyz.data(), cloud.data.size());
    cloud_pub_->publish(cloud);
  }

  gazebo::sensors::GpuRaySensorPtr sensor_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::event::ConnectionPtr connection_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  std::vector<ScanDirection> pattern_;
  std::string frame_id_;
  int samples_{20000};
  int downsample_{1};
  bool drop_invalid_{false};
  size_t start_{0};
};

GZ_REGISTER_SENSOR_PLUGIN(GpuLivoxPlugin)

}  // namespace gpu_livox_lidar
