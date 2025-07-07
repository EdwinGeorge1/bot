cmake_minimum_required(VERSION 3.8)
project(bot_trajectory)

# Enable warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nlohmann_json REQUIRED)

# Add executable
add_executable(trajectory_saver_node src/trajectory_saver_node.cpp)

# Link libraries and dependencies
ament_target_dependencies(trajectory_saver_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  std_srvs
  example_interfaces
)

target_link_libraries(trajectory_saver_node
  nlohmann_json::nlohmann_json
)

# Install
install(TARGETS
  trajectory_saver_node
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
