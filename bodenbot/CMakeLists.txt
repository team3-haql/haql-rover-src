cmake_minimum_required(VERSION 3.8)
project(bodenbot)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  bodenbot
  SHARED
  hardware/bodenbot.cpp
)

find_library(I2C_LIB i2c)
target_link_libraries(bodenbot PRIVATE ${I2C_LIB})

target_compile_features(bodenbot PUBLIC cxx_std_17)
target_include_directories(bodenbot PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/bodenbot>
)

ament_target_dependencies(
  bodenbot PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "BODENBOT_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface bodenbot.xml)

# INSTALL
# install(
#   DIRECTORY hardware/include/
#   DESTINATION include/${PROJECT_NAME}
# )
install(
  DIRECTORY bringup/launch bringup/config description/urdf description/ros2_control
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY description/meshes
  DESTINATION share/${PROJECT_NAME}/description
)

install(TARGETS bodenbot
  EXPORT export_bodenbot
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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

ament_package()
