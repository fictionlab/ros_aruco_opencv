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

#include "yaml-cpp/yaml.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "cv_bridge/cv_bridge.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/camera_common.hpp"

#include "aruco_opencv_msgs/msg/aruco_detection.hpp"
#include "aruco_opencv_msgs/msg/board_pose.hpp"

#include "aruco_opencv/utils.hpp"
#include "aruco_opencv/parameters.hpp"

using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace aruco_opencv
{

class ArucoTracker : public rclcpp_lifecycle::LifecycleNode
{
  // Parameters
  std::string cam_base_topic_;
  bool image_is_rectified_;
  std::string output_frame_;
  std::string marker_dict_;
  bool transform_poses_;
  bool publish_tf_;
  double marker_size_;
  bool image_sub_compressed_;
  int image_sub_qos_reliability_;
  int image_sub_qos_durability_;
  int image_sub_qos_depth_;
  std::string image_transport_;
  std::string board_descriptions_path_;

  // ROS
  OnSetParametersCallbackHandle::SharedPtr on_set_parameter_callback_handle_;
  rclcpp_lifecycle::LifecyclePublisher<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr
    detection_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_img_sub_;
  rclcpp::Time last_msg_stamp_;
  bool cam_info_retrieved_ = false;
  rclcpp::Time callback_start_time_;
  bool new_aruco_params_ = false;

  // Aruco
  cv::Mat camera_matrix_;
  cv::Mat distortion_coeffs_;
  cv::Mat marker_obj_points_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_parameters_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  std::vector<std::pair<std::string, cv::Ptr<cv::aruco::Board>>> boards_;

  // Thread safety
  std::mutex cam_info_mutex_;

  // Tf2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

public:
  explicit ArucoTracker(rclcpp::NodeOptions options)
  : LifecycleNode("aruco_tracker", options),
    camera_matrix_(3, 3, CV_64FC1),
    distortion_coeffs_(4, 1, CV_64FC1, cv::Scalar(0)),
    marker_obj_points_(4, 1, CV_32FC3)
  {
    declare_parameters();
  }

  LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Configuring");

    #if CV_VERSION_MAJOR > 4 || CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7
    detector_parameters_ = cv::makePtr<cv::aruco::DetectorParameters>();
    #else
    detector_parameters_ = cv::aruco::DetectorParameters::create();
    #endif

    retrieve_parameters();

    if (ARUCO_DICT_MAP.find(marker_dict_) == ARUCO_DICT_MAP.end()) {
      RCLCPP_ERROR_STREAM(get_logger(), "Unsupported dictionary name: " << marker_dict_);
      return LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    #if CV_VERSION_MAJOR > 4 || CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7
    dictionary_ = cv::makePtr<cv::aruco::Dictionary>(
      cv::aruco::getPredefinedDictionary(
        ARUCO_DICT_MAP.at(marker_dict_)));
    #else
    dictionary_ = cv::aruco::getPredefinedDictionary(ARUCO_DICT_MAP.at(marker_dict_));
    #endif

    if (!board_descriptions_path_.empty()) {
      load_boards();
    }

    update_marker_obj_points();

    if (publish_tf_) {
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    }

    detection_pub_ = create_publisher<aruco_opencv_msgs::msg::ArucoDetection>(
      "aruco_detections", 5);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>("~/debug", 5);

    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "Activating");

    if (transform_poses_) {
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    LifecycleNode::on_activate(state);

    detection_pub_->on_activate();
    debug_pub_->on_activate();

    on_set_parameter_callback_handle_ =
      add_on_set_parameters_callback(
      std::bind(
        &ArucoTracker::callback_on_set_parameters,
        this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Waiting for first camera info...");

    cam_info_retrieved_ = false;

    std::string image_topic = rclcpp::expand_topic_or_service_name(
      cam_base_topic_, this->get_name(), this->get_namespace());
    std::string cam_info_topic = image_transport::getCameraInfoTopic(image_topic);

    cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      cam_info_topic, 1,
      std::bind(&ArucoTracker::callback_camera_info, this, std::placeholders::_1));

    rmw_qos_profile_t image_sub_qos = rmw_qos_profile_default;
    image_sub_qos.reliability =
      static_cast<rmw_qos_reliability_policy_t>(image_sub_qos_reliability_);
    image_sub_qos.durability = static_cast<rmw_qos_durability_policy_t>(image_sub_qos_durability_);
    image_sub_qos.depth = image_sub_qos_depth_;

    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(image_sub_qos), image_sub_qos);

    if (image_sub_compressed_) {
      compressed_img_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        image_topic + "/compressed", qos, std::bind(
          &ArucoTracker::callback_compressed_image, this, std::placeholders::_1));
    } else {
      img_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic, qos, std::bind(
          &ArucoTracker::callback_image, this, std::placeholders::_1));
    }

    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "Deactivating");

    on_set_parameter_callback_handle_.reset();
    cam_info_sub_.reset();
    img_sub_.reset();
    compressed_img_sub_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();

    detection_pub_->on_deactivate();
    debug_pub_->on_deactivate();

    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Cleaning up");

    tf_broadcaster_.reset();
    dictionary_.reset();
    detector_parameters_.reset();
    detection_pub_.reset();
    debug_pub_.reset();
    boards_.clear();

    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "Shutting down");

    on_set_parameter_callback_handle_.reset();
    cam_info_sub_.reset();
    img_sub_.reset();
    compressed_img_sub_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    tf_broadcaster_.reset();
    dictionary_.reset();
    detector_parameters_.reset();
    detection_pub_.reset();
    debug_pub_.reset();
    boards_.clear();

    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

protected:
  void declare_parameters()
  {
    declare_param(*this, "cam_base_topic", "camera/image_raw");
    declare_param(*this, "image_is_rectified", false, false);
    declare_param(*this, "output_frame", "");
    declare_param(*this, "marker_dict", "4X4_50");
    declare_param(*this, "image_sub_compressed", false);
    declare_param(
      *this, "image_sub_qos.reliability",
      static_cast<int>(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT));
    declare_param(
      *this, "image_sub_qos.durability",
      static_cast<int>(RMW_QOS_POLICY_DURABILITY_VOLATILE));
    declare_param(*this, "image_sub_qos.depth", 1);
    declare_param(*this, "publish_tf", true, true);
    declare_param(*this, "marker_size", 0.15, true);
    declare_param(*this, "board_descriptions_path", "");

    declare_aruco_parameters(*this);
  }

  void retrieve_parameters()
  {
    get_param(*this, "cam_base_topic", cam_base_topic_, "Camera Base Topic: ");

    get_parameter("image_is_rectified", image_is_rectified_);
    RCLCPP_INFO_STREAM(
      get_logger(), "Assume images are rectified: " << (image_is_rectified_ ? "YES" : "NO"));

    get_parameter("output_frame", output_frame_);
    if (output_frame_.empty()) {
      RCLCPP_INFO(get_logger(), "Marker detections will be published in the camera frame");
      transform_poses_ = false;
    } else {
      RCLCPP_INFO(
        get_logger(), "Marker detections will be transformed to \'%s\' frame",
        output_frame_.c_str());
      transform_poses_ = true;
    }

    get_param(*this, "marker_dict", marker_dict_, "Marker Dictionary name: ");

    get_parameter("image_sub_compressed", image_sub_compressed_);

    get_parameter("image_sub_qos.reliability", image_sub_qos_reliability_);
    get_parameter("image_sub_qos.durability", image_sub_qos_durability_);
    get_parameter("image_sub_qos.depth", image_sub_qos_depth_);

    get_parameter("publish_tf", publish_tf_);
    RCLCPP_INFO_STREAM(get_logger(), "TF publishing is " << (publish_tf_ ? "enabled" : "disabled"));

    get_param(*this, "marker_size", marker_size_, "Marker size: ");

    get_parameter("board_descriptions_path", board_descriptions_path_);

    RCLCPP_INFO(get_logger(), "Aruco Parameters:");
    retrieve_aruco_parameters(*this, detector_parameters_, true);
  }

  rcl_interfaces::msg::SetParametersResult callback_on_set_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // Validate parameters
    for (auto & param : parameters) {
      if (param.get_name() == "marker_size") {
        if (param.as_double() <= 0.0) {
          result.successful = false;
          result.reason = param.get_name() + " must be positive";
          RCLCPP_ERROR_STREAM(get_logger(), result.reason);
          return result;
        }
      }
    }

    for (auto & param : parameters) {
      if (param.get_name() == "marker_size") {
        marker_size_ = param.as_double();
        update_marker_obj_points();
      } else if (param.get_name().rfind("aruco", 0) == 0) {
        new_aruco_params_ = true;
      } else {
        // Unknown parameter, ignore
        continue;
      }

      RCLCPP_INFO_STREAM(
        get_logger(),
        "Parameter \"" << param.get_name() << "\" changed to " << param.value_to_string());
    }

    return result;
  }

  void load_boards()
  {
    RCLCPP_INFO_STREAM(
      get_logger(), "Trying to load board descriptions from " << board_descriptions_path_);

    YAML::Node descriptions;
    try {
      descriptions = YAML::LoadFile(board_descriptions_path_);
    } catch (const YAML::Exception & e) {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to load board descriptions: " << e.what());
      return;
    }

    if (!descriptions.IsSequence()) {
      RCLCPP_ERROR(get_logger(), "Failed to load board descriptions: root node is not a sequence");
    }

    for (const YAML::Node & desc : descriptions) {
      std::string name;
      try {
        name = desc["name"].as<std::string>();
        const bool frame_at_center = desc["frame_at_center"].as<bool>();
        const int markers_x = desc["markers_x"].as<int>();
        const int markers_y = desc["markers_y"].as<int>();
        const double marker_size = desc["marker_size"].as<double>();
        const double separation = desc["separation"].as<double>();
        const int first_id = desc["first_id"].as<int>();

        #if CV_VERSION_MAJOR > 4 || CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7
        std::vector<int> ids(markers_x * markers_y);
        std::iota(ids.begin(), ids.end(), first_id);
        cv::Ptr<cv::aruco::Board> board = cv::makePtr<cv::aruco::GridBoard>(
          cv::Size(markers_x, markers_y), marker_size, separation, *dictionary_, ids);
        #else
        cv::Ptr<cv::aruco::Board> board = cv::aruco::GridBoard::create(
          markers_x, markers_y, marker_size, separation, dictionary_, first_id);
        #endif

        if (frame_at_center) {
          double offset_x = (markers_x * (marker_size + separation) - separation) / 2.0;
          double offset_y = (markers_y * (marker_size + separation) - separation) / 2.0;

          #if CV_VERSION_MAJOR > 4 || CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7
          std::vector<std::vector<cv::Point3f>> obj_points(board->getObjPoints());
          #else
          std::vector<std::vector<cv::Point3f>> obj_points(board->objPoints);
          #endif

          for (auto & obj : obj_points) {
            for (auto & point : obj) {
              point.x -= offset_x;
              point.y -= offset_y;
            }
          }

          // Create a new board with all the object point offsetted so that point (0,0)
          // is at the center of the board
          #if CV_VERSION_MAJOR > 4 || CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7
          board = cv::makePtr<cv::aruco::Board>(obj_points, *dictionary_, ids);
          #else
          board = cv::aruco::Board::create(obj_points, dictionary_, board->ids);
          #endif
        }

        boards_.push_back(std::make_pair(name, board));
      } catch (const YAML::Exception & e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Failed to load board '" << name << "': " << e.what());
        continue;
      }
      RCLCPP_INFO_STREAM(
        get_logger(), "Successfully loaded configuration for board '" << name << "'");
    }
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

  void callback_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr cam_info)
  {
    std::lock_guard<std::mutex> guard(cam_info_mutex_);

    if (image_is_rectified_) {
      for (int i = 0; i < 9; ++i) {
        camera_matrix_.at<double>(i / 3, i % 3) = cam_info->p[i + i / 3];
      }
    } else {
      for (int i = 0; i < 9; ++i) {
        camera_matrix_.at<double>(i / 3, i % 3) = cam_info->k[i];
      }
      distortion_coeffs_ = cv::Mat(cam_info->d, true);
    }

    if (!cam_info_retrieved_) {
      RCLCPP_INFO(get_logger(), "First camera info retrieved.");
      cam_info_retrieved_ = true;
    }
  }

  void callback_compressed_image(const sensor_msgs::msg::CompressedImage::ConstSharedPtr img_msg)
  {
    if (!should_process_img_msg(img_msg)) {
      return;
    }

    auto cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
    process_image(cv_ptr);
  }

  void callback_image(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
  {
    if (!should_process_img_msg(img_msg)) {
      return;
    }

    auto cv_ptr = cv_bridge::toCvShare(img_msg);
    process_image(cv_ptr);
  }

  template<typename ImgMsgT>
  bool should_process_img_msg(ImgMsgT img_msg)
  {
    RCLCPP_DEBUG_STREAM(get_logger(), "Image message address [SUBSCRIBE]:\t" << img_msg.get());

    if (!cam_info_retrieved_) {
      RCLCPP_DEBUG(get_logger(), "Camera info not retrieved yet. Ignoring image...");
      return false;
    }

    if (img_msg->header.stamp == last_msg_stamp_) {
      RCLCPP_DEBUG(
        get_logger(),
        "The new image has the same timestamp as the previous one (duplicate frame?). Ignoring...");
      return false;
    }

    last_msg_stamp_ = img_msg->header.stamp;

    // We're ready to go, remember the current time to measure callback performance.
    callback_start_time_ = get_clock()->now();

    return true;
  }

  void process_image(const cv_bridge::CvImageConstPtr & cv_ptr)
  {
    if (new_aruco_params_) {
      new_aruco_params_ = false;
      retrieve_aruco_parameters(*this, detector_parameters_);
    }

    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;

    // TODO(bjsowa): mutex
    cv::aruco::detectMarkers(
      cv_ptr->image, dictionary_, marker_corners, marker_ids,
      detector_parameters_);

    int n_markers = marker_ids.size();
    std::vector<cv::Vec3d> rvec_final(n_markers), tvec_final(n_markers);

    aruco_opencv_msgs::msg::ArucoDetection detection;
    detection.header.frame_id = cv_ptr->header.frame_id;
    detection.header.stamp = cv_ptr->header.stamp;
    detection.markers.resize(n_markers);

    {
      std::lock_guard<std::mutex> guard(cam_info_mutex_);

      cv::parallel_for_(
        cv::Range(0, n_markers), [&](const cv::Range & range) {
          for (size_t i = range.start; i < range.end; i++) {
            int id = marker_ids[i];

            cv::solvePnP(
              marker_obj_points_, marker_corners[i], camera_matrix_, distortion_coeffs_,
              rvec_final[i], tvec_final[i], false, cv::SOLVEPNP_IPPE_SQUARE);

            detection.markers[i].marker_id = id;
            detection.markers[i].pose = convert_rvec_tvec(rvec_final[i], tvec_final[i]);
          }
        });

      for (const auto & board_desc : boards_) {
        std::string name = board_desc.first;
        auto & board = board_desc.second;

        cv::Vec3d rvec, tvec;
        int valid = cv::aruco::estimatePoseBoard(
          marker_corners, marker_ids, board, camera_matrix_,
          distortion_coeffs_, rvec, tvec);

        if (valid > 0) {
          aruco_opencv_msgs::msg::BoardPose bpose;
          bpose.board_name = name;
          bpose.pose = convert_rvec_tvec(rvec, tvec);
          detection.boards.push_back(bpose);
          rvec_final.push_back(rvec);
          tvec_final.push_back(tvec);
          n_markers++;
        }
      }
    }

    if (transform_poses_ && n_markers > 0) {
      detection.header.frame_id = output_frame_;
      geometry_msgs::msg::TransformStamped cam_to_output;
      // Retrieve camera -> output_frame transform
      try {
        cam_to_output = tf_buffer_->lookupTransform(
          output_frame_, cv_ptr->header.frame_id,
          cv_ptr->header.stamp, rclcpp::Duration::from_seconds(1.0));
      } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR_STREAM(get_logger(), ex.what());
        return;
      }
      for (auto & marker_pose : detection.markers) {
        tf2::doTransform(marker_pose.pose, marker_pose.pose, cam_to_output);
      }
      for (auto & board_pose : detection.boards) {
        tf2::doTransform(board_pose.pose, board_pose.pose, cam_to_output);
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
      for (auto & board_pose : detection.boards) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = detection.header.stamp;
        transform.header.frame_id = detection.header.frame_id;
        transform.child_frame_id = std::string("board_") + board_pose.board_name;
        tf2::Transform tf_transform;
        tf2::fromMsg(board_pose.pose, tf_transform);
        transform.transform = tf2::toMsg(tf_transform);
        transforms.push_back(transform);
      }
      tf_broadcaster_->sendTransform(transforms);
    }

    detection_pub_->publish(detection);

    if (debug_pub_->get_subscription_count() > 0) {
      auto debug_cv_ptr = std::make_shared<cv_bridge::CvImage>();
      debug_cv_ptr->header = cv_ptr->header;
      debug_cv_ptr->encoding = cv_ptr->encoding;
      debug_cv_ptr->image = cv_ptr->image.clone();
      cv::aruco::drawDetectedMarkers(debug_cv_ptr->image, marker_corners, marker_ids);
      {
        std::lock_guard<std::mutex> guard(cam_info_mutex_);
        for (size_t i = 0; i < n_markers; i++) {
          cv::drawFrameAxes(
            debug_cv_ptr->image, camera_matrix_, distortion_coeffs_, rvec_final[i],
            tvec_final[i], 0.2, 3);
        }
      }
      std::unique_ptr<sensor_msgs::msg::Image> debug_img =
        std::make_unique<sensor_msgs::msg::Image>();
      debug_cv_ptr->toImageMsg(*debug_img);
      debug_pub_->publish(std::move(debug_img));
    }

    auto callback_end_time = get_clock()->now();
    double whole_callback_duration = (callback_end_time - callback_start_time_).seconds();
    double image_send_duration = (callback_start_time_ - cv_ptr->header.stamp).seconds();

    RCLCPP_DEBUG(
      get_logger(), "Image callback completed. The callback started %.4f s after the image"
      " frame was grabbed and completed its execution in %.4f s.", image_send_duration,
      whole_callback_duration);
  }
};

class ArucoTrackerAutostart : public ArucoTracker
{
public:
  explicit ArucoTrackerAutostart(rclcpp::NodeOptions options)
  : ArucoTracker(options)
  {
    auto new_state = configure();
    if (new_state.label() == "inactive") {
      activate();
    }
  }
};

}  // namespace aruco_opencv

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(aruco_opencv::ArucoTracker)
RCLCPP_COMPONENTS_REGISTER_NODE(aruco_opencv::ArucoTrackerAutostart)
