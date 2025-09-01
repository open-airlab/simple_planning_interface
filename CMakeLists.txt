cmake_minimum_required(VERSION 3.12.0)
project(simple_planning_interface)

# find_package(ament REQUIRED COMPONENTS interactive_markers rclcpp visualization_msgs tf2_ros)
find_package(ament_cmake REQUIRED)
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
set(INCLUDE_DIRS include 
  ${EIGEN3_INCLUDE_DIRS}
)
include_directories(${INCLUDE_DIRS})


# === Build library from basic_interface.cpp ===
# add Library
add_library(basic_interface SHARED
  src/basic_interface.cpp
)

# Link libraries
ament_target_dependencies(basic_interface
  rclcpp
  interactive_markers
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)

# === Add Executable(s) ===

# Source files
# set(SOURCES
#   src/basic_interface.cpp
#   src/basic_interface_node.cpp
# )
# add_executable(basic_interface ${SOURCES})

# Create executable

add_executable(basic_interface_node 
  src/basic_interface_node.cpp
)
target_link_libraries(basic_interface_node
  basic_interface
)

###########
## Install ##
###########

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install libraries
install(TARGETS basic_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS 
  basic_interface_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
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

# # Export libraries - this is crucial for other packages to find the library

ament_export_include_directories(${INCLUDE_DIRS})

ament_export_libraries(basic_interface)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

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