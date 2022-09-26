cmake_minimum_required(VERSION 3.5)
project(aruco_opencv)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(aruco_opencv_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/single_marker_tracker.cpp
)
target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
)
ament_target_dependencies(${PROJECT_NAME}
  aruco_opencv_msgs
  rclcpp_components
  cv_bridge
  image_transport
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "aruco_opencv::SingleMarkerTracker"
  EXECUTABLE "single_marker_tracker"
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
