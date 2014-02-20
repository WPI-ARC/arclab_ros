cmake_minimum_required(VERSION 2.8.3)
project(lidar_aggregator)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS geometry_msgs laser_geometry pcl_conversions message_generation roscpp sensor_msgs std_msgs tf)
find_package(PCL REQUIRED)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate services in the 'srv' folder
add_service_files(DIRECTORY srv FILES LidarAggregation.srv)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

###################################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS geometry_msgs laser_geometry pcl_conversions message_runtime roscpp sensor_msgs std_msgs tf DEPENDS system_lib PCL)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(lidar_aggregator_node src/lidar_aggregator_node.cpp)
add_dependencies(lidar_aggregator_node ${catkin_EXPORTED_TARGETS} lidar_aggregator_gencpp)
target_link_libraries(lidar_aggregator_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS lidar_aggregator_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
