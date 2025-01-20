################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(cps_rmp220_support)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
  roscpp
  rospy
  std_msgs
  actionlib
  #move_base_msgs
  tf
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS urdf xacro
)

################################################################################
# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_publisher nodes/odom_publisher.cpp)
target_link_libraries(odom_publisher ${catkin_LIBRARIES})

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
#catkin_install_python(PROGRAMS
#  nodes/odom_publisher_python.py
#  nodes/odom_publisher_linus.py
#  nodes/odom_publisher_fromTwist.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

# Find all Python scripts in the nodes/ directory
file(GLOB python_scripts nodes/*.py)

# Mark all found scripts as executable and install them
catkin_install_python(PROGRAMS ${python_scripts}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


################################################################################
# Install
################################################################################
install(DIRECTORY meshes rviz urdf config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install description files
install(DIRECTORY description
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.xacro" PATTERN "*.stl"
)

# Install launch files
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch*"
)

# Install world files
install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.world"
)

# Install miscellaneous files like README, LICENSE, etc.
install(FILES README.md LICENSE.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install configuration files
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.yaml"
)
################################################################################
# Test
################################################################################