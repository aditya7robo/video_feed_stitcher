cmake_minimum_required(VERSION 3.0.2)
project(video_feed_stitcher)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
  sensor_msgs

  diagnostic_msgs
  dynamic_reconfigure
  geometry_msgs
  image_geometry
  message_generation
  pluginlib
  rosbuild
  rviz
  seven_robotics_msgs
  seven_robot_utils
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

###################################
## catkin-specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp image_transport cv_bridge sensor_msgs diagnostic_msgs dynamic_reconfigure geometry_msgs image_geometry 
                  message_generation pluginlib rosbuild rviz seven_robotics_msgs seven_robot_utils std_msgs
   DEPENDS OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/video_feed_stitcher.cpp)

## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ## Mark other files for installation (e.g., launch and bag files, etc.)
# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )