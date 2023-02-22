^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aruco_opencv
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Fix build for Debian Buster
* Contributors: Błażej Sowa

0.3.0 (2023-02-22)
------------------
* Add Board detection (`#6 <https://github.com/fictionlab/aruco_opencv/issues/6>`_)
  * Rename SingleMarkerTracker to ArucoTracker
  * Add BoardPose msg, change MarkerDetection to ArucoDetection
  * Load board descriptions from yaml file
  * Add more boards to example configuration
  * Change default marker dictionary
  * Add board pose estimation
  * Lock camera info for board estimation
* Ignore duplicate image frames
* Add scripts for generating markers and boards
* Fix included headers file extensions
* Port changes from foxy branch
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
