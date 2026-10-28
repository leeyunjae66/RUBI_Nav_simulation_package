cmake_minimum_required(VERSION 3.8)
project(rubi_control_core)

find_package(ament_cmake REQUIRED)

set(ONNXRUNTIME_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/onnxruntime")
set(ONNXRUNTIME_LIBRARY "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so")

if(NOT EXISTS "${ONNXRUNTIME_ROOT}/include/onnxruntime_cxx_api.h")
  message(FATAL_ERROR "Vendored ONNX Runtime headers are missing")
endif()
if(NOT EXISTS "${ONNXRUNTIME_LIBRARY}")
  message(FATAL_ERROR "Vendored ONNX Runtime library is missing")
endif()

add_library(onnxruntime_core SHARED IMPORTED GLOBAL)
set_target_properties(onnxruntime_core PROPERTIES
  IMPORTED_LOCATION "${ONNXRUNTIME_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_ROOT}/include"
)

add_library(${PROJECT_NAME} SHARED
  src/controller.cpp
  src/gazebo_legacy_policy_adapter.cpp
  src/gazebo_legacy_policy_runner.cpp
  src/gazebo_terrain_policy_adapter.cpp
  src/gazebo_terrain_policy_runner.cpp
  src/policy_runner.cpp
  src/rubi_w_policy_adapter.cpp
  src/rubi_w_policy_runner.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PRIVATE onnxruntime_core)
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${ONNXRUNTIME_ROOT}/lib"
  INSTALL_RPATH "$ORIGIN"
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/
  DESTINATION include
)
install(FILES
  models/encoder.onnx
  models/policy.onnx
  models/rubi_gazebo_legacy_policy.onnx
  models/rubi_w_encoder.onnx
  models/rubi_w_policy.onnx
  DESTINATION share/${PROJECT_NAME}/models
)
install(DIRECTORY models/rubi_plane_onnx
  DESTINATION share/${PROJECT_NAME}/models
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY third_party/onnxruntime/lib/
  DESTINATION lib
  FILES_MATCHING PATTERN "libonnxruntime.so*"
)
install(FILES third_party/onnxruntime/LICENSE
  DESTINATION share/${PROJECT_NAME}/vendor/onnxruntime
)
install(EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake
  NAMESPACE ${PROJECT_NAME}::
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  add_executable(rubi_control_core_test test/control_core_test.cpp)
  target_compile_features(rubi_control_core_test PRIVATE cxx_std_17)
  target_link_libraries(rubi_control_core_test PRIVATE ${PROJECT_NAME})
  target_compile_definitions(rubi_control_core_test PRIVATE
    RUBI_ENCODER_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/models/rubi_plane_onnx/encoder.onnx"
    RUBI_POLICY_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/models/rubi_plane_onnx/policy.onnx"
  )

  set(_core_test_cases
    canonical_onnx_contract
    deterministic_zero_synthetic_inference
    observation_30d_offsets_and_scales
    identity_quaternion_projected_gravity
    rotated_quaternion_projected_gravity
    joint_position_relative_default
    history_oldest_to_newest_shift
    encoder_input_dimension_300
    policy_concatenation_dimension_36
    action_dimension_6
    ten_physics_ticks_two_inferences
    action_hold_between_inferences
    walk_ready_duration_0_5_sec
    torque_off_same_tick_exact_zero
    emergency_stop_same_tick_exact_zero
    torque_clamp_plus_minus_90
    nan_state_safe_zero
    reset_clears_history_action_phase
    wrong_joint_manifest_fail_fast
    missing_corrupt_policy_fail_fast
  )
  foreach(_case IN LISTS _core_test_cases)
    add_test(NAME core_${_case} COMMAND rubi_control_core_test --case ${_case})
    set_tests_properties(core_${_case} PROPERTIES TIMEOUT 20)
  endforeach()

  add_executable(rubi_gazebo_legacy_core_test
    test/gazebo_legacy_policy_adapter_test.cpp)
  target_compile_features(rubi_gazebo_legacy_core_test PRIVATE cxx_std_17)
  target_link_libraries(rubi_gazebo_legacy_core_test PRIVATE ${PROJECT_NAME})
  target_compile_definitions(rubi_gazebo_legacy_core_test PRIVATE
    RUBI_GAZEBO_LEGACY_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/models/rubi_gazebo_legacy_policy.onnx"
    RUBI_CANONICAL_POLICY_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/models/policy.onnx"
  )
  set(_legacy_core_test_cases
    onnx_contract_320_to_7
    deterministic_finite_inference
    observation_32d_offsets_scales
    history_oldest_to_newest
    phase_and_cycle_output_order
    action_mapping_and_scale
    policy_decimation_five
    walk_ready_source_contract
    torque_off_same_tick_zero
    emergency_stop_same_tick_zero
    nan_state_same_tick_zero
    reset_clears_history_action_phase
    wrong_canonical_policy_fail_fast
  )
  foreach(_case IN LISTS _legacy_core_test_cases)
    add_test(NAME legacy_gazebo_${_case}
      COMMAND rubi_gazebo_legacy_core_test --case ${_case})
    set_tests_properties(legacy_gazebo_${_case} PROPERTIES TIMEOUT 20)
  endforeach()

  add_executable(rubi_gazebo_terrain_core_test
    test/gazebo_terrain_policy_adapter_test.cpp)
  target_compile_features(rubi_gazebo_terrain_core_test PRIVATE cxx_std_17)
  target_link_libraries(rubi_gazebo_terrain_core_test PRIVATE ${PROJECT_NAME})
  target_compile_definitions(rubi_gazebo_terrain_core_test PRIVATE
    RUBI_TERRAIN_ENCODER_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/models/encoder.onnx"
    RUBI_TERRAIN_POLICY_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/models/policy.onnx"
  )
  set(_terrain_core_test_cases
    onnx_contract_330_to_32_65_to_6
    observation_actor_history_and_policy_order
    history_oldest_to_newest
    action_torque_limit_clipping
    policy_decimation_five
    policy_before_walk_ready_is_rejected
    reset_and_nonfinite_safety
  )
  foreach(_case IN LISTS _terrain_core_test_cases)
    add_test(NAME terrain_gazebo_${_case}
      COMMAND rubi_gazebo_terrain_core_test --case ${_case})
    set_tests_properties(terrain_gazebo_${_case} PROPERTIES TIMEOUT 20)
  endforeach()

  add_executable(rubi_w_control_core_test
    test/rubi_w_policy_adapter_test.cpp)
  target_compile_features(rubi_w_control_core_test PRIVATE cxx_std_17)
  target_link_libraries(rubi_w_control_core_test PRIVATE ${PROJECT_NAME})
  target_compile_definitions(rubi_w_control_core_test PRIVATE
    RUBI_W_ENCODER_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/models/rubi_w_encoder.onnx"
    RUBI_W_POLICY_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/models/rubi_w_policy.onnx"
    RUBI_W_WRONG_ENCODER_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/test/models/wrong_280_encoder.onnx"
    RUBI_W_WRONG_POLICY_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/test/models/wrong_35_policy.onnx"
  )
  set(_rubi_w_core_test_cases
    onnx_contract_340_to_3_40_to_8
    deterministic_finite_inference
    observation_34d_offsets_scales
    history_340_oldest_to_newest
    policy_concatenation_40
    action_mapping_8
    leg_position_wheel_velocity_split
    action_scale_source_contract
    leg_wheel_gains_source_contract
    policy_decimation_five
    torque_off_estop_same_tick_eight_zero
    nonfinite_same_tick_eight_zero
    reset_clears_history_action_phase
    six_dof_manifest_fail_fast
    wrong_280_35_8_pair_fail_fast
  )
  foreach(_case IN LISTS _rubi_w_core_test_cases)
    add_test(NAME rubi_w_${_case}
      COMMAND rubi_w_control_core_test --case ${_case})
    set_tests_properties(rubi_w_${_case} PROPERTIES TIMEOUT 20)
  endforeach()
endif()

ament_package()
