cmake_minimum_required(VERSION 3.0.2)
project(aruco_opencv)

find_package(catkin REQUIRED COMPONENTS
  aruco_opencv_msgs
  cv_bridge
  dynamic_reconfigure
  image_transport
  nodelet
  roscpp
  tf2_geometry_msgs
  tf2_ros
)

generate_dynamic_reconfigure_options(
  cfg/ArucoDetector.cfg
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/single_marker_tracker.cpp
  src/utils.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}
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
