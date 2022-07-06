cmake_minimum_required(VERSION 3.0.2)
project(aruco_opencv)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_transport
  message_generation
  nodelet
  roscpp
  std_msgs
  tf2_geometry_msgs
  tf2_ros
)

generate_dynamic_reconfigure_options(
  cfg/ArucoDetector.cfg
)

add_message_files(
  DIRECTORY msg
  FILES
    MarkerDetection.msg
    MarkerPose.msg
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/single_marker_tracker.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencfg
)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
