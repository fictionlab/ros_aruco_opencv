// Copyright 2022 Kell Ideas sp. z o.o.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <mutex>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/camera_common.h>
#include <image_transport/image_transport.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <aruco_opencv/ArucoDetectorConfig.h>
#include <aruco_opencv/utils.hpp>
#include <aruco_opencv_msgs/MarkerDetection.h>

namespace aruco_opencv {

class ArucoTracker : public nodelet::Nodelet {

  // Parameters
  std::string cam_base_topic_;
  std::string output_frame_;
  std::string marker_dict_;
  bool transform_poses_;
  bool publish_tf_;
  double marker_size_;
  int image_queue_size_;

  // ROS
  ros::Publisher detection_pub_;
  ros::Subscriber cam_info_sub_;
  ros::Time last_msg_stamp;
  bool cam_info_retrieved_ = false;
  image_transport::ImageTransport *it_;
  image_transport::ImageTransport *pit_;
  image_transport::Subscriber img_sub_;
  image_transport::Publisher debug_pub_;
  dynamic_reconfigure::Server<aruco_opencv::ArucoDetectorConfig> *dyn_srv_;

  // Aruco
  cv::Mat camera_matrix_;
  cv::Mat distortion_coeffs_;
  cv::Mat marker_obj_points_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_parameters_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;

  // Thread safety
  std::mutex cam_info_mutex_;

  // Tf2
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener *tf_listener_;
  tf2_ros::TransformBroadcaster *tf_broadcaster_;

public:
  ArucoTracker()
      : camera_matrix_(3, 3, CV_64FC1), distortion_coeffs_(4, 1, CV_64FC1),
        marker_obj_points_(4, 1, CV_32FC3) {}

private:
  void onInit() override {
    auto &nh = getNodeHandle();
    auto &pnh = getPrivateNodeHandle();

    detector_parameters_ = cv::aruco::DetectorParameters::create();

    retrieve_parameters(pnh);

    dyn_srv_ = new dynamic_reconfigure::Server<aruco_opencv::ArucoDetectorConfig>(pnh);
    dyn_srv_->setCallback(boost::bind(&ArucoTracker::reconfigure_callback, this, _1, _2));

    if (transform_poses_)
      tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);

    if (publish_tf_)
      tf_broadcaster_ = new tf2_ros::TransformBroadcaster();

    if (ARUCO_DICT_MAP.find(marker_dict_) == ARUCO_DICT_MAP.end()) {
      ROS_ERROR_STREAM("Unsupported dictionary name: " << marker_dict_);
      return;
    }

    dictionary_ = cv::aruco::getPredefinedDictionary(ARUCO_DICT_MAP.at(marker_dict_));

    // set coordinate system in the middle of the marker, with Z pointing out
    marker_obj_points_.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-marker_size_ / 2.f, marker_size_ / 2.f, 0);
    marker_obj_points_.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(marker_size_ / 2.f, marker_size_ / 2.f, 0);
    marker_obj_points_.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(marker_size_ / 2.f, -marker_size_ / 2.f, 0);
    marker_obj_points_.ptr<cv::Vec3f>(0)[3] =
        cv::Vec3f(-marker_size_ / 2.f, -marker_size_ / 2.f, 0);

    it_ = new image_transport::ImageTransport(nh);
    pit_ = new image_transport::ImageTransport(pnh);

    detection_pub_ = nh.advertise<aruco_opencv_msgs::MarkerDetection>("marker_detections", 5);
    debug_pub_ = pit_->advertise("debug", 1);

    NODELET_INFO("Waiting for first camera info...");

    std::string cam_info_topic = image_transport::getCameraInfoTopic(cam_base_topic_);
    cam_info_sub_ =
        nh.subscribe(cam_info_topic, 1, &ArucoTracker::callback_camera_info, this);

    img_sub_ = it_->subscribe(cam_base_topic_, image_queue_size_,
                              &ArucoTracker::callback_image, this);
  }

  void retrieve_parameters(ros::NodeHandle &pnh) {
    pnh.param<std::string>("cam_base_topic", cam_base_topic_, "camera/image_raw");
    ROS_INFO_STREAM("Camera Base Topic: " << cam_base_topic_);

    pnh.param<std::string>("output_frame", output_frame_, "");
    if (output_frame_.empty()) {
      ROS_INFO("Marker detections will be published in the camera frame");
      transform_poses_ = false;
    } else {
      ROS_INFO("Marker detections will be transformed to \'%s\' frame", output_frame_.c_str());
      transform_poses_ = true;
    }

    pnh.param<std::string>("marker_dict", marker_dict_, "ARUCO_ORIGINAL");
    ROS_INFO_STREAM("Marker Dictionary name: " << marker_dict_);

    pnh.param<bool>("publish_tf", publish_tf_, true);
    ROS_INFO_STREAM("TF publishing is " << (publish_tf_ ? "enabled" : "disabled"));

    pnh.param<double>("marker_size", marker_size_, 0.15);
    ROS_INFO_STREAM("Marker size: " << marker_size_);

    pnh.param<int>("image_queue_size", image_queue_size_, 1);
    ROS_INFO_STREAM("Image Queue size: " << image_queue_size_);
  }

  void reconfigure_callback(aruco_opencv::ArucoDetectorConfig &config, uint32_t level) {
    if (config.adaptiveThreshWinSizeMax < config.adaptiveThreshWinSizeMin)
      config.adaptiveThreshWinSizeMax = config.adaptiveThreshWinSizeMin;

    if (config.maxMarkerPerimeterRate < config.minMarkerPerimeterRate)
      config.maxMarkerPerimeterRate = config.minMarkerPerimeterRate;

    detector_parameters_->adaptiveThreshWinSizeMin = config.adaptiveThreshWinSizeMin;
    detector_parameters_->adaptiveThreshWinSizeMax = config.adaptiveThreshWinSizeMax;
    detector_parameters_->adaptiveThreshWinSizeStep = config.adaptiveThreshWinSizeStep;
    detector_parameters_->adaptiveThreshConstant = config.adaptiveThreshConstant;
    detector_parameters_->minMarkerPerimeterRate = config.minMarkerPerimeterRate;
    detector_parameters_->maxMarkerPerimeterRate = config.maxMarkerPerimeterRate;
    detector_parameters_->polygonalApproxAccuracyRate = config.polygonalApproxAccuracyRate;
    detector_parameters_->minCornerDistanceRate = config.minCornerDistanceRate;
    detector_parameters_->minDistanceToBorder = config.minDistanceToBorder;
    detector_parameters_->minMarkerDistanceRate = config.minMarkerDistanceRate;
    detector_parameters_->markerBorderBits = config.markerBorderBits;
    detector_parameters_->perspectiveRemovePixelPerCell = config.perspectiveRemovePixelPerCell;
    detector_parameters_->perspectiveRemoveIgnoredMarginPerCell =
        config.perspectiveRemoveIgnoredMarginPerCell;
    detector_parameters_->maxErroneousBitsInBorderRate = config.maxErroneousBitsInBorderRate;
    detector_parameters_->minOtsuStdDev = config.minOtsuStdDev;
    detector_parameters_->errorCorrectionRate = config.errorCorrectionRate;
#if CV_VERSION_MAJOR >= 4
    detector_parameters_->cornerRefinementMethod = config.cornerRefinementMethod;
#else
    detector_parameters_->doCornerRefinement = config.cornerRefinementMethod == 1;
#endif
    detector_parameters_->cornerRefinementWinSize = config.cornerRefinementWinSize;
    detector_parameters_->cornerRefinementMaxIterations = config.cornerRefinementMaxIterations;
    detector_parameters_->cornerRefinementMinAccuracy = config.cornerRefinementMinAccuracy;
  }

  void callback_camera_info(const sensor_msgs::CameraInfo &cam_info) {
    std::lock_guard<std::mutex> guard(cam_info_mutex_);

    for (int i = 0; i < 9; ++i)
      camera_matrix_.at<double>(i / 3, i % 3) = cam_info.K[i];
    for (int i = 0; i < 4; ++i)
      distortion_coeffs_.at<double>(i, 0) = cam_info.D[i];

    if (!cam_info_retrieved_) {
      NODELET_INFO("First camera info retrieved.");
      cam_info_retrieved_ = true;
    }
  }

  void callback_image(const sensor_msgs::ImageConstPtr &img_msg) {
    ROS_DEBUG_STREAM("Image message address [SUBSCRIBE]:\t" << img_msg.get());

    if (!cam_info_retrieved_)
      return;

    if (img_msg->header.stamp == last_msg_stamp) {
      ROS_DEBUG("The new image has the same timestamp as the previous one (duplicate frame?). "
                "Ignoring...");
      return;
    }
    last_msg_stamp = img_msg->header.stamp;

    auto callback_start_time = ros::Time::now();

    // Convert the image
    auto cv_ptr = cv_bridge::toCvShare(img_msg);

    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;

    cv::aruco::detectMarkers(cv_ptr->image, dictionary_, marker_corners, marker_ids,
                             detector_parameters_);

    int n_markers = marker_ids.size();
    std::vector<cv::Vec3d> rvec_final(n_markers), tvec_final(n_markers);

    aruco_opencv_msgs::MarkerDetection detection;
    detection.header.frame_id = img_msg->header.frame_id;
    detection.header.stamp = img_msg->header.stamp;
    detection.markers.resize(n_markers);

    cam_info_mutex_.lock();
    cv::parallel_for_(cv::Range(0, n_markers), [&](const cv::Range &range) {
      for (int i = range.start; i < range.end; i++) {
        int id = marker_ids[i];

#if CV_VERSION_MAJOR >= 4
        cv::solvePnP(marker_obj_points_, marker_corners[i], camera_matrix_, distortion_coeffs_,
                     rvec_final[i], tvec_final[i], false, cv::SOLVEPNP_IPPE_SQUARE);
#else
        cv::solvePnP(marker_obj_points_, marker_corners[i], camera_matrix_, distortion_coeffs_,
                     rvec_final[i], tvec_final[i], false, cv::SOLVEPNP_ITERATIVE);
#endif

        detection.markers[i].marker_id = id;
        detection.markers[i].pose = convert_rvec_tvec(rvec_final[i], tvec_final[i]);
      }
    });
    cam_info_mutex_.unlock();

    if (transform_poses_ && n_markers > 0) {
      detection.header.frame_id = output_frame_;
      geometry_msgs::TransformStamped cam_to_output;
      // Retrieve camera -> output_frame transform
      try {
        cam_to_output = tf_buffer_.lookupTransform(output_frame_, img_msg->header.frame_id,
                                                   img_msg->header.stamp, ros::Duration(1.0));
      } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return;
      }
      for (auto &marker_pose : detection.markers)
        tf2::doTransform(marker_pose.pose, marker_pose.pose, cam_to_output);
    }

    if (publish_tf_ && n_markers > 0) {
      std::vector<geometry_msgs::TransformStamped> transforms;
      for (auto &marker_pose : detection.markers) {
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = detection.header.stamp;
        transform.header.frame_id = detection.header.frame_id;
        transform.child_frame_id = std::string("marker_") + std::to_string(marker_pose.marker_id);
        tf2::Transform tf_transform;
        tf2::fromMsg(marker_pose.pose, tf_transform);
        transform.transform = tf2::toMsg(tf_transform);
        transforms.push_back(transform);
      }
      tf_broadcaster_->sendTransform(transforms);
    }

    detection_pub_.publish(detection);

    if (debug_pub_.getNumSubscribers() > 0) {
      auto debug_cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
      cv::aruco::drawDetectedMarkers(debug_cv_ptr->image, marker_corners, marker_ids);
      for (size_t i = 0; i < marker_ids.size(); i++) {
#if CV_VERSION_MAJOR >= 4
        cv::drawFrameAxes(debug_cv_ptr->image, camera_matrix_, distortion_coeffs_, rvec_final[i],
                          tvec_final[i], 0.2, 3);
#else
        cv::aruco::drawAxis(debug_cv_ptr->image, camera_matrix_, distortion_coeffs_, rvec_final[i],
                            tvec_final[i], 0.2);
#endif
      }

      debug_pub_.publish(debug_cv_ptr->toImageMsg());
    }

    auto callback_end_time = ros::Time::now();
    double whole_callback_duration = (callback_end_time - callback_start_time).toSec();
    double image_send_duration = (callback_start_time - img_msg->header.stamp).toSec();

    NODELET_DEBUG("Image callback completed. The callback started %.4f s after "
                  "the image frame was "
                  "grabbed and completed its execution in %.4f s.",
                  image_send_duration, whole_callback_duration);
  }
};

} // namespace aruco_opencv

PLUGINLIB_EXPORT_CLASS(aruco_opencv::ArucoTracker, nodelet::Nodelet)
