// Copyright 2025 Fictionlab sp. z o.o.
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

#pragma once

#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "aruco_opencv/utils.hpp"
#include "aruco_opencv/parameters.hpp"

namespace aruco_opencv
{

struct MarkerPose
{
  int marker_id;
  geometry_msgs::msg::Pose pose;
};

struct BoardPoseOut
{
  std::string board_name;
  geometry_msgs::msg::Pose pose;
};

class ArucoDetector {
public:
  ArucoDetector() = delete;
  explicit ArucoDetector(rclcpp::Logger logger = rclcpp::get_logger("ArucoDetector"));

  void set_dictionary(const std::string & dictionary_name);
  void set_detector_parameters(const DetectorParams & params);
  void set_aruco_parameters(const cv::Ptr<cv::aruco::DetectorParameters> & params);
  void set_camera_intrinsics(const cv::Mat & camera_matrix, const cv::Mat & dist_coeffs);
  void update_camera_info(const sensor_msgs::msg::CameraInfo & cam_info, bool image_is_rectified);
  void get_intrinsics(cv::Mat & camera_matrix, cv::Mat & dist_coeffs) const;
  void set_boards(const std::vector<std::pair<std::string, cv::Ptr<cv::aruco::Board>>> & boards);
  cv::Ptr<cv::aruco::Dictionary> get_dictionary();

  void detect(
    const cv::Mat & image,
    std::vector<int> & marker_ids,
    std::vector<std::vector<cv::Point2f>> & marker_corners) const;

  void estimate_marker_poses(
    const std::vector<int> & marker_ids,
    const std::vector<std::vector<cv::Point2f>> & marker_corners,
    std::vector<MarkerPose> & marker_poses,
    std::vector<cv::Vec3d> & rvecs,
    std::vector<cv::Vec3d> & tvecs) const;

  void estimate_board_poses(
    const std::vector<int> & marker_ids,
    const std::vector<std::vector<cv::Point2f>> & marker_corners,
    std::vector<BoardPoseOut> & board_poses,
    std::vector<cv::Vec3d> & rvecs,
    std::vector<cv::Vec3d> & tvecs) const;

private:
  void update_marker_object_points(double marker_size);

  /**
   * @brief Selects the best pose from multiple candidates based on the given strategy
   * @param rvecs rotation vectors of candidate poses
   * @param tvecs translation vectors of candidate poses
   * @param reproj_errors reprojection errors of candidate poses
   * @param selector_config configuration for pose selection
   * @return index of the selected pose
   */
  size_t select_pose_from_candidates(
    const std::vector<cv::Vec3d> & rvecs,
    const std::vector<cv::Vec3d> & tvecs,
    const std::vector<double> & reproj_errors,
    const PoseSelectorConfig & selector_config) const;

  rclcpp::Logger logger_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> aruco_parameters_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_coeffs_;
  cv::Mat marker_obj_points_;
  std::vector<std::pair<std::string, cv::Ptr<cv::aruco::Board>>> boards_;

  DetectorParams params_{};

  mutable std::mutex intrinsics_mutex_;
};

}  // namespace aruco_opencv
