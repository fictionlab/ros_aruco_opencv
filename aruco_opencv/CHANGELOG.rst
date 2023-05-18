^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aruco_opencv
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.1.1 (2023-05-18)
------------------
* Fix create_marker and create_board script permissions (`#22 <https://github.com/fictionlab/ros_aruco_opencv/issues/22>`_)
* Contributors: Błażej Sowa

4.1.0 (2023-02-22)
------------------
* Add python dependencies (`#19 <https://github.com/fictionlab/ros_aruco_opencv/issues/19>`_)
* Add board detection (ROS2) (`#16 <https://github.com/fictionlab/ros_aruco_opencv/issues/16>`_)
  * Rename SingleMarkerTracker to ArucoTracker
  * Add BoardPose msg, change MarkerDetection to ArucoDetection
  * Change default marker dictionary
  * Add board descriptions
  * Add board pose estimation
  * Fix cpplint errors
* Add scripts for generating markers and boards (`#13 <https://github.com/fictionlab/ros_aruco_opencv/issues/13>`_)
* Ignore duplicate image frames (`#10 <https://github.com/fictionlab/ros_aruco_opencv/issues/10>`_)
* Add ament_lint tests to cmakelists instead of github workflows (`#7 <https://github.com/fictionlab/ros_aruco_opencv/issues/7>`_)
* Contributors: Błażej Sowa

4.0.1 (2022-12-13)
------------------

4.0.0 (2022-12-12)
------------------
* Better camera calibration handling (`#3 <https://github.com/fictionlab/ros_aruco_opencv/issues/3>`_)
  * Support different distortion models
  * Support rectified images
* Use newer headers for cv_bridge
* Use the custom QoS for image subscription
* Fix autostart node activation
* Use newer headers for tf2_geometry_msgs
* Fix aruco library linking
* Fix build for Humble
* Remove image_transport parameter
* Add marker_dict parameter
* Unsubscribe from image topic on shutdown
* Add single_marker_tracker_autostart node
* Reformat code with uncrustify
* Use cv::parallel_for\_ in PnP pose computation
* Refactor parameter declaration and retrieval, add utils.hpp
* Initial LifecycleNode implementation
* Update project dependencies
* Add copyright notice
* Install config directory
* Port launch and config file to ROS2
* Remove old dynamic reconfigure config
* Declare the rest of the parameters
* Initial port for ROS2 Foxy
* Simplify filling the camera matrix from camera info
* Contributors: Błażej Sowa

0.2.0 (2022-09-07)
------------------
* Move message definitions to aruco_opencv_msgs package (`#2 <https://github.com/fictionlab/aruco_opencv/issues/2>`_)
* Fix build for Debian Buster
* Publish transforms from output_frame to markers on tf2
* Transform marker poses to specified output frame
* Allow changing camera info without restarting the tracker
* Add dynamically reconfigurable parameters for corner refinement
* Contributors: Błażej Sowa

0.1.0 (2022-07-07)
------------------
* Initial version of the package
* Contributors: Błażej Sowa
