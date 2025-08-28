cmake_minimum_required(VERSION 3.12.0)
project(simple_planning_interface)

# find_package(ament REQUIRED COMPONENTS interactive_markers rclcpp visualization_msgs tf2_ros)
find_package(rclcpp REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  # ROS == Indigo.
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
else()
  # ROS > Indigo.
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


###########
## Build ##
###########

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Source files
set(SOURCES
  src/basic_interface.cpp
  src/basic_interface_node.cpp
)

# Create executable
add_executable(basic_interface ${SOURCES})

# Link libraries
ament_target_dependencies(basic_interface
  rclcpp
  interactive_markers
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)

###########
## Install ##
###########

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install executables
install(TARGETS basic_interface
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
# **Install launch files**
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY cfg/
  DESTINATION share/${PROJECT_NAME}/cfg
)
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  interactive_markers
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Finalize package
ament_package()