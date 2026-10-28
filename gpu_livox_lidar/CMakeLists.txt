cmake_minimum_required(VERSION 3.8)
project(gpu_livox_lidar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

add_library(gpu_livox_plugin SHARED src/gpu_livox_plugin.cpp)
target_include_directories(gpu_livox_plugin PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(gpu_livox_plugin ${GAZEBO_LIBRARIES})
ament_target_dependencies(gpu_livox_plugin gazebo_dev gazebo_ros rclcpp sensor_msgs
  livox_ros_driver2)

install(TARGETS gpu_livox_plugin LIBRARY DESTINATION lib)

ament_package()
