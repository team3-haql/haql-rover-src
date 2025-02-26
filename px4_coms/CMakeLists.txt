cmake_minimum_required(VERSION 3.8)
project(px4_coms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(
  px4_coms_component SHARED
  src/px4_remote_control_subscriber.cpp
  src/px4_waypoint_publisher.cpp
)

ament_target_dependencies(
  px4_coms_component SYSTEM
  px4_msgs
  rclcpp
)

target_link_libraries(
  px4_coms_component
  zigmaps
)

target_include_directories(px4_coms_component PUBLIC include)

ament_package()
