^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aruco_opencv
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2022-12-12)
------------------
* Better camera calibration handling (`#3 <https://github.com/fictionlab/aruco_opencv/issues/3>`_) (`#5 <https://github.com/fictionlab/aruco_opencv/issues/5>`_)
  * Support different distortion models
  * Support rectified images
* Use the custom QoS for image subscription
* Fix autostart node activation
* Fix build for Humble
* Remove image_transport parameter
* Add marker_dict parameter
* Unsubscribe from image topic on shutdown
* Add single_marker_tracker_autostart node
* Reformat code with uncrustify
* Use cv::parallel_for_ in PnP pose computation
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
