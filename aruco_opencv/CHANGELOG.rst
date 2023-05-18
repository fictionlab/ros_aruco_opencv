^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aruco_opencv
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Fix create_marker and create_board script permissions (`#22 <https://github.com/fictionlab/aruco_opencv/issues/22>`_) (`#24 <https://github.com/fictionlab/aruco_opencv/issues/24>`_)
* Contributors: Błażej Sowa

2.1.0 (2023-02-22)
------------------
* Add python dependencies (`#19 <https://github.com/fictionlab/aruco_opencv/issues/19>`_) (`#21 <https://github.com/fictionlab/aruco_opencv/issues/21>`_)
* Add board detection (ROS2) (backport `#16 <https://github.com/fictionlab/aruco_opencv/issues/16>`_) (`#18 <https://github.com/fictionlab/aruco_opencv/issues/18>`_)
  * Rename SingleMarkerTracker to ArucoTracker
  * Add BoardPose msg, change MarkerDetection to ArucoDetection
  * Change default marker dictionary
  * Add board descriptions
  * Add board pose estimation
  * Fix cpplint errors
* Add scripts for generating markers and boards (`#13 <https://github.com/fictionlab/aruco_opencv/issues/13>`_) (`#15 <https://github.com/fictionlab/aruco_opencv/issues/15>`_)
* Ignore duplicate image frames (`#10 <https://github.com/fictionlab/aruco_opencv/issues/10>`_) (`#12 <https://github.com/fictionlab/aruco_opencv/issues/12>`_)
* Add ament_lint tests to cmakelists instead of github workflows (backport `#7 <https://github.com/fictionlab/aruco_opencv/issues/7>`_) (`#8 <https://github.com/fictionlab/aruco_opencv/issues/8>`_)
* Contributors: Błażej Sowa

2.0.1 (2022-12-14)
------------------

2.0.0 (2022-12-12)
------------------
* Better camera calibration handling (`#3 <https://github.com/fictionlab/aruco_opencv/issues/3>`_) (`#4 <https://github.com/fictionlab/aruco_opencv/issues/4>`_)
  * Support different distortion models
  * Support rectified images
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
