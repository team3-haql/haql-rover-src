cmake_minimum_required(VERSION 3.8)
project(traverse_layer)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -g)
endif()

find_package(ament_cmake REQUIRED)
find_package(grid_map_cmake_helpers REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(map_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
# grid_map_packages()

if(TARGET tf2_geometry_msgs::tf2_geometry_msgs)
  get_target_property(_include_dirs tf2_geometry_msgs::tf2_geometry_msgs INTERFACE_INCLUDE_DIRECTORIES)
else()
  set(_include_dirs ${tf2_geometry_msgs_INCLUDE_DIRS})
endif()

find_file(TF2_CPP_HEADERS
  NAMES tf2_geometry_msgs.hpp
  PATHS ${_include_dirs}
  NO_CACHE
  PATH_SUFFIXES tf2_geometry_msgs
)

set(dependencies
    grid_map_msgs
    grid_map_ros
    tf2_ros
    tf2_sensor_msgs
    tf2_geometry_msgs
    tf2
    sensor_msgs
    map_msgs
    geometry_msgs
    cv_bridge
    rclcpp_components
    rclcpp)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
    include
    SYSTEM
    ${EIGEN3_INCLUDE_DIRS}
)

# TraverseLayer
add_library(
    traverse_layer_component SHARED
    src/traverse_layer_component.cpp
)
ament_target_dependencies(
    traverse_layer_component SYSTEM
    ${dependencies}
)
target_link_libraries(
    traverse_layer_component
    zigmaps
)

rclcpp_components_register_node(
    traverse_layer_component
    PLUGIN "traverse_layer::TraverseLayer"
    EXECUTABLE traverse_layer
)

# PointcloudToGridmap
add_library(
    pointcloud_to_gridmap_component SHARED
    src/pointcloud_to_gridmap_component.cpp
)
ament_target_dependencies(
    pointcloud_to_gridmap_component SYSTEM
    ${dependencies}
)

rclcpp_components_register_node(
    pointcloud_to_gridmap_component
    PLUGIN "traverse_layer::PointcloudToGridmap"
    EXECUTABLE pointcloud_to_gridmap
)

set(targets_list
    traverse_layer_component
    pointcloud_to_gridmap_component
)

# foreach(target ${targets_list})
#   if(CMAKE_COMPILER_IS_GNUCXX)
#     target_compile_options(${target} PRIVATE "SHELL:--param ggc-min-expand=1")
#     target_compile_options(${target} PRIVATE "SHELL:--param ggc-min-heapsize=32768")
#   endif()
# endforeach()

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
    TARGETS ${targets_list}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Mark other files for installation
install(
  DIRECTORY config launch data
  DESTINATION share/${PROJECT_NAME}
)


#############
## Testing ##
#############

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

ament_export_include_directories(include ${EIGEN3_INCLUDE_DIRS})
ament_package()
