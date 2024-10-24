cmake_minimum_required(VERSION 3.0.2)
project(steered_diff_drive_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    controller_interface
    control_msgs
    ackermann_msgs
    dynamic_reconfigure
    geometry_msgs
    hardware_interface
    nav_msgs
    pluginlib
    realtime_tools
    tf
    urdf
)

find_package(Boost REQUIRED)

# Declare a catkin package
generate_dynamic_reconfigure_options(cfg/SteeredDiffDriveController.cfg)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    controller_interface
    control_msgs
    ackermann_msgs
    dynamic_reconfigure
    geometry_msgs
    hardware_interface
    nav_msgs
    realtime_tools
    tf
  LIBRARIES ${PROJECT_NAME}
  DEPENDS Boost
)


###########
## Build ##
###########

# Specify header include paths
include_directories(
  include ${catkin_INCLUDE_DIRS}
  include ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/steered_diff_drive_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)


#############
## Install ##
#############

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install plugins
install(FILES steered_drive_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
