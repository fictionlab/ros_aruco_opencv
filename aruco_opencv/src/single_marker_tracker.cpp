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
#include <chrono>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include "rclcpp/rclcpp.hpp"

#include "cv_bridge/cv_bridge.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

// #include <image_transport/camera_common.h>
#include "image_transport/camera_common.hpp"
#include "image_transport/image_transport.hpp"

#include "aruco_opencv_msgs/msg/marker_detection.hpp"

namespace aruco_opencv
{

inline geometry_msgs::msg::Pose convert_rvec_tvec(const cv::Vec3d & rvec, const cv::Vec3d & tvec)
{
  geometry_msgs::msg::Pose pose_out;

  cv::Mat rot(3, 3, CV_64FC1);
  cv::Rodrigues(rvec, rot);

  tf2::Matrix3x3 tf_rot(rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2),
    rot.at<double>(1, 0), rot.at<double>(1, 1), rot.at<double>(1, 2),
    rot.at<double>(2, 0), rot.at<double>(2, 1), rot.at<double>(2, 2));
  tf2::Quaternion tf_quat;
  tf_rot.getRotation(tf_quat);

  pose_out.position.x = tvec[0];
  pose_out.position.y = tvec[1];
  pose_out.position.z = tvec[2];
  tf2::convert(tf_quat, pose_out.orientation);

  return pose_out;
}

class SingleMarkerTracker : public rclcpp::Node
{

  // Parameters
  std::string cam_base_topic_;
  std::string output_frame_;
  bool transform_poses_;
  bool publish_tf_;
  double marker_size_;
  int image_sub_qos_reliability_;
  int image_sub_qos_durability_;
  int image_sub_qos_depth_;
  std::string image_transport_;

  // ROS
  OnSetParametersCallbackHandle::SharedPtr on_set_parameter_callback_handle_;
  rclcpp::Publisher<aruco_opencv_msgs::msg::MarkerDetection>::SharedPtr detection_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  bool cam_info_retrieved_ = false;
  image_transport::Subscriber img_sub_;
  image_transport::Publisher debug_pub_;

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
  tf2_ros::TransformListener * tf_listener_;
  tf2_ros::TransformBroadcaster * tf_broadcaster_;

public:
  SingleMarkerTracker(rclcpp::NodeOptions options)
  : Node("single_marker_tracker", options), camera_matrix_(3, 3, CV_64FC1),
    distortion_coeffs_(4, 1, CV_64FC1),
    marker_obj_points_(4, 1, CV_32FC3),
    tf_buffer_(get_clock())
  {
    detector_parameters_ = cv::aruco::DetectorParameters::create();
    // TODO: Add parameter for dictionary
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

    retrieve_parameters();

    if (transform_poses_) {
      tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
    }

    if (publish_tf_) {
      tf_broadcaster_ = new tf2_ros::TransformBroadcaster(*this);
    }

    update_marker_obj_points();

    on_set_parameter_callback_handle_ =
      add_on_set_parameters_callback(
      std::bind(
        &SingleMarkerTracker::callback_on_set_parameters,
        this, std::placeholders::_1));

    detection_pub_ = create_publisher<aruco_opencv_msgs::msg::MarkerDetection>(
      "marker_detections", 5);
    debug_pub_ = image_transport::create_publisher(this, "~/debug");

    RCLCPP_INFO(get_logger(), "Waiting for first camera info...");

    std::string cam_info_topic = image_transport::getCameraInfoTopic(cam_base_topic_);
    cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      cam_info_topic, 1,
      std::bind(&SingleMarkerTracker::callback_camera_info, this, std::placeholders::_1));

    rmw_qos_profile_t image_sub_qos = rmw_qos_profile_default;
    image_sub_qos.reliability =
      static_cast<rmw_qos_reliability_policy_t>(image_sub_qos_reliability_);
    image_sub_qos.durability = static_cast<rmw_qos_durability_policy_t>(image_sub_qos_durability_);
    image_sub_qos.depth = image_sub_qos_depth_;

    img_sub_ =
      image_transport::create_subscription(
      this, cam_base_topic_,
      std::bind(
        &SingleMarkerTracker::callback_image, this,
        std::placeholders::_1), image_transport_, image_sub_qos);
  }

private:
  template<typename T>
  void get_param(
    std::string param_name, T & out_value, T default_value, std::string log_info = "",
    bool dynamic = false)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = !dynamic;

    declare_parameter(param_name, rclcpp::ParameterValue(default_value), descriptor);
    get_parameter(param_name, out_value);

    if (!log_info.empty()) {
      RCLCPP_INFO_STREAM(get_logger(), log_info << out_value);
    }
  }

  void get_param_int_range(
    std::string param_name, int & out_value, int default_value,
    int min_value, int max_value, std::string log_info = "")
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    // descriptor.integer_range.push_back()
    auto range = rcl_interfaces::msg::IntegerRange();
    range.from_value = min_value;
    range.to_value = max_value;

    descriptor.integer_range.push_back(range);

    declare_parameter(param_name, default_value, descriptor);
    get_parameter(param_name, out_value);

    if (!log_info.empty()) {
      RCLCPP_INFO_STREAM(get_logger(), log_info << out_value);
    }
  }

  void get_param_double_range(
    std::string param_name, double & out_value, double default_value,
    double min_value, double max_value, std::string log_info = "")
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    // descriptor.integer_range.push_back()
    auto range = rcl_interfaces::msg::FloatingPointRange();
    range.from_value = min_value;
    range.to_value = max_value;

    descriptor.floating_point_range.push_back(range);

    declare_parameter(param_name, default_value, descriptor);
    get_parameter(param_name, out_value);

    if (!log_info.empty()) {
      RCLCPP_INFO_STREAM(get_logger(), log_info << out_value);
    }
  }

  void retrieve_parameters()
  {
    get_param<std::string>(
      "cam_base_topic", cam_base_topic_, "camera/image_raw",
      "Camera Base Topic: ");

    get_param<std::string>("output_frame", output_frame_, "");
    if (output_frame_.empty()) {
      RCLCPP_INFO(get_logger(), "Marker detections will be published in the camera frame");
      transform_poses_ = false;
    } else {
      RCLCPP_INFO(
        get_logger(), "Marker detections will be transformed to \'%s\' frame", output_frame_);
      transform_poses_ = true;
    }

    get_param<std::string>("image_transport", image_transport_, "raw", "Image transport: ");

    get_param(
      "image_sub_qos.reliability", image_sub_qos_reliability_,
      static_cast<int>(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT));
    get_param(
      "image_sub_qos.durability", image_sub_qos_durability_,
      static_cast<int>(RMW_QOS_POLICY_DURABILITY_VOLATILE));
    get_param(
      "image_sub_qos.depth", image_sub_qos_depth_, 1);

    get_param("publish_tf", publish_tf_, true, "", true);
    RCLCPP_INFO_STREAM(get_logger(), "TF publishing is " << (publish_tf_ ? "enabled" : "disabled"));

    get_param("marker_size", marker_size_, 0.15, "Marker size: ", true);

    RCLCPP_INFO(get_logger(), "Aruco Parameters:");
    get_param_int_range(
      "aruco.adaptiveThreshWinSizeMin",
      detector_parameters_->adaptiveThreshWinSizeMin, 3, 3, 100, " * adaptiveThreshWinSizeMin: ");
    get_param_int_range(
      "aruco.adaptiveThreshWinSizeMax",
      detector_parameters_->adaptiveThreshWinSizeMax, 23, 3, 100, " * adaptiveThreshWinSizeMax: ");
    get_param_int_range(
      "aruco.adaptiveThreshWinSizeStep",
      detector_parameters_->adaptiveThreshWinSizeMin, 10, 1, 100, " * adaptiveThreshWinSizeStep: ");
    get_param_double_range(
      "aruco.adaptiveThreshConstant",
      detector_parameters_->adaptiveThreshConstant, 7.0, 0.0, 100.0, " * adaptiveThreshConstant: ");
    get_param_double_range(
      "aruco.minMarkerPerimeterRate",
      detector_parameters_->minMarkerPerimeterRate, 0.03, 0.0, 4.0, " * minMarkerPerimeterRate: ");
    get_param_double_range(
      "aruco.maxMarkerPerimeterRate",
      detector_parameters_->maxMarkerPerimeterRate, 4.0, 0.0, 4.0, " * maxMarkerPerimeterRate: ");
    get_param_double_range(
      "aruco.polygonalApproxAccuracyRate",
      detector_parameters_->polygonalApproxAccuracyRate, 0.03, 0.0, 0.3,
      " * polygonalApproxAccuracyRate: ");
    get_param_double_range(
      "aruco.minCornerDistanceRate",
      detector_parameters_->minCornerDistanceRate, 0.05, 0.0, 0.25, " * minCornerDistanceRate: ");
    get_param_int_range(
      "aruco.minDistanceToBorder",
      detector_parameters_->minDistanceToBorder, 3, 0, 100, " * minDistanceToBorder: ");
    get_param_double_range(
      "aruco.minMarkerDistanceRate",
      detector_parameters_->minMarkerDistanceRate, 0.05, 0.0, 0.25, " * minMarkerDistanceRate: ");
    get_param_int_range(
      "aruco.markerBorderBits", detector_parameters_->markerBorderBits, 1, 1, 3,
      " * markerBorderBits: ");
    get_param_int_range(
      "aruco.perspectiveRemovePixelPerCell",
      detector_parameters_->perspectiveRemovePixelPerCell, 4, 1, 20,
      " * perspectiveRemovePixelPerCell: ");
    get_param_double_range(
      "aruco.perspectiveRemoveIgnoredMarginPerCell",
      detector_parameters_->perspectiveRemoveIgnoredMarginPerCell, 0.13, 0.0, 0.5,
      " * perspectiveRemoveIgnoredMarginPerCell: ");
    get_param_double_range(
      "aruco.maxErroneousBitsInBorderRate",
      detector_parameters_->maxErroneousBitsInBorderRate, 0.35, 0.0, 1.0,
      " * maxErroneousBitsInBorderRate: ");
    get_param_double_range(
      "aruco.minOtsuStdDev",
      detector_parameters_->minOtsuStdDev, 5.0, 0.0, 30.0, " * minOtsuStdDev: ");
    get_param_double_range(
      "aruco.errorCorrectionRate",
      detector_parameters_->errorCorrectionRate, 0.6, 0.0, 1.0, " * errorCorrectionRate: ");
    get_param_int_range(
      "aruco.cornerRefinementMethod", detector_parameters_->cornerRefinementMethod, 2, 0, 2);
    std::map<int, std::string> crmethod = {{0, "NONE"}, {1, "SUBPIX"}, {2, "CONTOUR"}};
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * cornerRefinementMethod: " << detector_parameters_->cornerRefinementMethod << " (" <<
        crmethod[detector_parameters_->cornerRefinementMethod] << ")");
    get_param_int_range(
      "aruco.cornerRefinementWinSize",
      detector_parameters_->cornerRefinementWinSize, 5, 2, 10, " * cornerRefinementWinSize: ");
    get_param_int_range(
      "aruco.cornerRefinementMaxIterations",
      detector_parameters_->cornerRefinementMaxIterations, 30, 1, 100,
      " * cornerRefinementMaxIterations: ");
    get_param_double_range(
      "aruco.cornerRefinementMinAccuracy",
      detector_parameters_->cornerRefinementMinAccuracy, 0.1, 0.01, 1.0,
      " * cornerRefinementMinAccuracy: ");
  }

  rcl_interfaces::msg::SetParametersResult callback_on_set_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // Validate parameters
    for (auto & param: parameters) {
      if (param.get_name() == "marker_size") {
        if (param.as_double() <= 0.0) {
          result.successful = false;
          result.reason = param.get_name() + " must be positive";
          RCLCPP_ERROR(get_logger(), result.reason);
          return result;
        }
      }
    }

    bool aruco_param_changed = false;
    for (auto & param : parameters) {
      if (param.get_name() == "marker_size") {
        marker_size_ = param.as_double();
        update_marker_obj_points();
      } else if (param.get_name().rfind("aruco", 0) == 0) {
        aruco_param_changed = true;
      } else {
        // Unknown parameter, ignore
        continue;
      }

      RCLCPP_INFO_STREAM(
        get_logger(),
        "Parameter \"" << param.get_name() << "\" changed to " << param.value_to_string());
    }

    if (!aruco_param_changed) {
      return result;
    }

    get_parameter("aruco.adaptiveThreshWinSizeMin", detector_parameters_->adaptiveThreshWinSizeMin);
    get_parameter("aruco.adaptiveThreshWinSizeMax", detector_parameters_->adaptiveThreshWinSizeMax);
    get_parameter(
      "aruco.adaptiveThreshWinSizeStep",
      detector_parameters_->adaptiveThreshWinSizeStep);
    get_parameter("aruco.adaptiveThreshConstant", detector_parameters_->adaptiveThreshConstant);
    get_parameter("aruco.minMarkerPerimeterRate", detector_parameters_->minMarkerPerimeterRate);
    get_parameter("aruco.maxMarkerPerimeterRate", detector_parameters_->maxMarkerPerimeterRate);
    get_parameter(
      "aruco.polygonalApproxAccuracyRate",
      detector_parameters_->polygonalApproxAccuracyRate);
    get_parameter("aruco.minCornerDistanceRate", detector_parameters_->minCornerDistanceRate);
    get_parameter("aruco.minDistanceToBorder", detector_parameters_->minDistanceToBorder);
    get_parameter("aruco.minMarkerDistanceRate", detector_parameters_->minMarkerDistanceRate);
    get_parameter("aruco.markerBorderBits", detector_parameters_->markerBorderBits);
    get_parameter(
      "aruco.perspectiveRemovePixelPerCell",
      detector_parameters_->perspectiveRemovePixelPerCell);
    get_parameter(
      "aruco.perspectiveRemoveIgnoredMarginPerCell",
      detector_parameters_->perspectiveRemoveIgnoredMarginPerCell);
    get_parameter(
      "aruco.maxErroneousBitsInBorderRate",
      detector_parameters_->maxErroneousBitsInBorderRate);
    get_parameter("aruco.minOtsuStdDev", detector_parameters_->minOtsuStdDev);
    get_parameter("aruco.errorCorrectionRate", detector_parameters_->errorCorrectionRate);
#if CV_VERSION_MAJOR >= 4
    get_parameter("aruco.cornerRefinementMethod", detector_parameters_->cornerRefinementMethod);
#else
    get_parameter("aruco.doCornerRefinement", detector_parameters_->doCornerRefinement);
#endif
    get_parameter("aruco.cornerRefinementWinSize", detector_parameters_->cornerRefinementWinSize);
    get_parameter(
      "aruco.cornerRefinementMaxIterations",
      detector_parameters_->cornerRefinementMaxIterations);
    get_parameter(
      "aruco.cornerRefinementMinAccuracy",
      detector_parameters_->cornerRefinementMinAccuracy);

    return result;
  }

  void update_marker_obj_points()
  {
    // set coordinate system in the middle of the marker, with Z pointing out
    marker_obj_points_.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-marker_size_ / 2.f, marker_size_ / 2.f, 0);
    marker_obj_points_.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(marker_size_ / 2.f, marker_size_ / 2.f, 0);
    marker_obj_points_.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(marker_size_ / 2.f, -marker_size_ / 2.f, 0);
    marker_obj_points_.ptr<cv::Vec3f>(0)[3] =
      cv::Vec3f(-marker_size_ / 2.f, -marker_size_ / 2.f, 0);
  }

  void callback_camera_info(const sensor_msgs::msg::CameraInfo::UniquePtr cam_info)
  {
    std::lock_guard<std::mutex> guard(cam_info_mutex_);

    for (int i = 0; i < 9; ++i) {
      camera_matrix_.at<double>(i / 3, i % 3) = cam_info->k[i];
    }
    for (int i = 0; i < 4; ++i) {
      distortion_coeffs_.at<double>(i, 0) = cam_info->d[i];
    }

    if (!cam_info_retrieved_) {
      RCLCPP_INFO(get_logger(), "First camera info retrieved.");
      cam_info_retrieved_ = true;
    }
  }

  void callback_image(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg)
  {
    if (!cam_info_retrieved_) {
      return;
    }

    auto callback_start_time = get_clock()->now();

    // Convert the image
    auto cv_ptr = cv_bridge::toCvShare(img_msg);

    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;

    // TODO: mutex
    cv::aruco::detectMarkers(
      cv_ptr->image, dictionary_, marker_corners, marker_ids,
      detector_parameters_);

    int n_markers = marker_ids.size();
    std::vector<cv::Vec3d> rvec_final(n_markers), tvec_final(n_markers);

    aruco_opencv_msgs::msg::MarkerDetection detection;
    detection.header.frame_id = img_msg->header.frame_id;
    detection.header.stamp = img_msg->header.stamp;

    cam_info_mutex_.lock();
    for (size_t i = 0; i < n_markers; i++) {
      int id = marker_ids[i];

#if CV_VERSION_MAJOR >= 4
      cv::solvePnP(
        marker_obj_points_, marker_corners[i], camera_matrix_, distortion_coeffs_,
        rvec_final[i], tvec_final[i], false, cv::SOLVEPNP_IPPE_SQUARE);
#else
      cv::solvePnP(
        marker_obj_points_, marker_corners[i], camera_matrix_, distortion_coeffs_,
        rvec_final[i], tvec_final[i], false, cv::SOLVEPNP_ITERATIVE);
#endif

      aruco_opencv_msgs::msg::MarkerPose mpose;
      mpose.marker_id = id;
      mpose.pose = convert_rvec_tvec(rvec_final[i], tvec_final[i]);
      detection.markers.push_back(mpose);
    }
    cam_info_mutex_.unlock();

    if (transform_poses_ && n_markers > 0) {
      detection.header.frame_id = output_frame_;
      geometry_msgs::msg::TransformStamped cam_to_output;
      // Retrieve camera -> output_frame transform
      try {
        cam_to_output = tf_buffer_.lookupTransform(
          output_frame_, img_msg->header.frame_id,
          img_msg->header.stamp, rclcpp::Duration::from_seconds(1.0));
      } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR_STREAM(get_logger(), ex.what());
        return;
      }
      for (auto & marker_pose : detection.markers) {
        tf2::doTransform(marker_pose.pose, marker_pose.pose, cam_to_output);
      }
    }

    if (publish_tf_ && n_markers > 0) {
      std::vector<geometry_msgs::msg::TransformStamped> transforms;
      for (auto & marker_pose : detection.markers) {
        geometry_msgs::msg::TransformStamped transform;
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

    detection_pub_->publish(detection);

    if (debug_pub_.getNumSubscribers() > 0) {
      auto debug_cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
      cv::aruco::drawDetectedMarkers(debug_cv_ptr->image, marker_corners, marker_ids);
      for (size_t i = 0; i < marker_ids.size(); i++) {
#if CV_VERSION_MAJOR >= 4
        cv::drawFrameAxes(
          debug_cv_ptr->image, camera_matrix_, distortion_coeffs_, rvec_final[i],
          tvec_final[i], 0.2, 3);
#else
        cv::aruco::drawAxis(
          debug_cv_ptr->image, camera_matrix_, distortion_coeffs_, rvec_final[i],
          tvec_final[i], 0.2);
#endif
      }

      debug_pub_.publish(debug_cv_ptr->toImageMsg());
    }

    auto callback_end_time = get_clock()->now();
    double whole_callback_duration = (callback_end_time - callback_start_time).seconds();
    double image_send_duration = (callback_start_time - img_msg->header.stamp).seconds();

    RCLCPP_DEBUG(
      get_logger(), "Image callback completed. The callback started %.4f s after the image"
      " frame was grabbed and completed its execution in %.4f s.", image_send_duration,
      whole_callback_duration);
  }
};

} // namespace aruco_opencv

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(aruco_opencv::SingleMarkerTracker)
