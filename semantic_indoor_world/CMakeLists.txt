cmake_minimum_required(VERSION 3.8)
project(semantic_indoor_world NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds config
  DESTINATION share/${PROJECT_NAME})

# `ros2 run semantic_indoor_world <script>`
install(PROGRAMS
  scripts/validate_semantic_map.py
  scripts/semantic_map_loader.py
  scripts/generate_world.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/world_layout.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_semantic_map test/test_semantic_map.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
