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

#include "aruco_opencv/detector.hpp"
#include "aruco_opencv/parameters.hpp"

#include <opencv2/calib3d.hpp>

namespace aruco_opencv
{

ArucoDetector::ArucoDetector()
: camera_matrix_(3, 3, CV_64FC1),
  distortion_coeffs_(4, 1, CV_64FC1, cv::Scalar(0)),
  marker_obj_points_(4, 1, CV_32FC3)
{}

void ArucoDetector::set_dictionary(const std::string & dictionary_name)
{
  #if CV_VERSION_MAJOR > 4 || CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7
  dictionary_ = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(
    ARUCO_DICT_MAP.at(dictionary_name)));
  #else
  dictionary_ = cv::aruco::getPredefinedDictionary(ARUCO_DICT_MAP.at(dictionary_name));
  #endif
}

void ArucoDetector::set_detector_parameters(const DetectorParams & params)
{
  params_ = params;
  this->update_marker_object_points(params_.marker_size);
}

void ArucoDetector::set_aruco_parameters(const cv::Ptr<cv::aruco::DetectorParameters> & params)
{
  aruco_parameters_ = params;
}

void ArucoDetector::update_marker_object_points(double marker_size)
{
  marker_obj_points_.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-marker_size / 2.f, marker_size / 2.f, 0);
  marker_obj_points_.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(marker_size / 2.f, marker_size / 2.f, 0);
  marker_obj_points_.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(marker_size / 2.f, -marker_size / 2.f, 0);
  marker_obj_points_.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-marker_size / 2.f, -marker_size / 2.f, 0);
}

void ArucoDetector::set_camera_intrinsics(
  const cv::Mat & camera_matrix,
  const cv::Mat & dist_coeffs)
{
  std::lock_guard<std::mutex> lk(intrinsics_mutex_);
  camera_matrix.copyTo(camera_matrix_);
  dist_coeffs.copyTo(distortion_coeffs_);
}

void ArucoDetector::update_camera_info(
  const sensor_msgs::msg::CameraInfo & cam_info,
  bool image_is_rectified)
{
  std::lock_guard<std::mutex> lk(intrinsics_mutex_);
  if (image_is_rectified) {
    for (int i = 0; i < 9; ++i) {
      camera_matrix_.at<double>(i / 3, i % 3) = cam_info.p[i + i / 3];
    }
    // For rectified images, distortion is assumed zero or already handled; keep current or zero.
  } else {
    for (int i = 0; i < 9; ++i) {
      camera_matrix_.at<double>(i / 3, i % 3) = cam_info.k[i];
    }
    distortion_coeffs_ = cv::Mat(cam_info.d, true);
  }
}

void ArucoDetector::get_intrinsics(cv::Mat & camera_matrix, cv::Mat & dist_coeffs) const
{
  std::lock_guard<std::mutex> lk(intrinsics_mutex_);
  camera_matrix_.copyTo(camera_matrix);
  distortion_coeffs_.copyTo(dist_coeffs);
}

void ArucoDetector::set_boards(
  const std::vector<std::pair<std::string,
  cv::Ptr<cv::aruco::Board>>> & boards)
{
  boards_ = boards;
}

cv::Ptr<cv::aruco::Dictionary> ArucoDetector::get_dictionary()
{
  std::lock_guard<std::mutex> lk(intrinsics_mutex_);
  return dictionary_;
}

void ArucoDetector::detect(
  const cv::Mat & image,
  std::vector<int> & marker_ids,
  std::vector<std::vector<cv::Point2f>> & marker_corners) const
{
  cv::aruco::detectMarkers(image, dictionary_, marker_corners, marker_ids, aruco_parameters_);
}

static geometry_msgs::msg::Pose select_pose_from_candidates(
  const std::vector<cv::Vec3d> & rvecs,
  const std::vector<cv::Vec3d> & tvecs,
  const std::vector<double> & reproj_errors,
  const PoseSelectorConfig & selector_config)
{
  if (rvecs.empty() || tvecs.empty() || reproj_errors.empty()) {
    return geometry_msgs::msg::Pose();
  }

  size_t best_index = 0;

  if (selector_config.strategy == PoseSelectorStrategy::REPROJECTION_ERROR) {
    double min_reproj_error = reproj_errors[0];
    for (size_t i = 1; i < reproj_errors.size(); ++i) {
      if (reproj_errors[i] < min_reproj_error) {
        min_reproj_error = reproj_errors[i];
        best_index = i;
      }
    }
  } else if (selector_config.strategy == PoseSelectorStrategy::PLANE_NORMAL) {
    // Select pose with the Z axis most aligned with camera Z axis (smallest angle)
    double max_cosine = -1.0;
    for (size_t i = 0; i < rvecs.size(); ++i) {
      cv::Mat R;
      cv::Rodrigues(rvecs[i], R);
      cv::Vec3d z_axis = R.col(2);
      double cosine = z_axis[2] / cv::norm(z_axis);
      if (cosine > max_cosine) {
        max_cosine = cosine;
        best_index = i;
      }
    }
  } else {
    // Default to first pose if strategy is unrecognized
    best_index = 0;
  }

  return convert_rvec_tvec(rvecs[best_index], tvecs[best_index]);
}

void ArucoDetector::estimate_marker_poses(
  const std::vector<int> & marker_ids,
  const std::vector<std::vector<cv::Point2f>> & marker_corners,
  std::vector<MarkerPose> & marker_poses,
  std::vector<cv::Vec3d> & rvecs,
  std::vector<cv::Vec3d> & tvecs) const
{
  const int n = static_cast<int>(marker_ids.size());
  rvecs.resize(n);
  tvecs.resize(n);
  marker_poses.resize(n);

  cv::Mat camera_matrix, distortion_coeffs;
  {
    std::lock_guard<std::mutex> lk(intrinsics_mutex_);
    camera_matrix_.copyTo(camera_matrix);
    distortion_coeffs_.copyTo(distortion_coeffs);
  }

  cv::parallel_for_(cv::Range(0, n), [&](const cv::Range & range) {
      for (int i = range.start; i < range.end; ++i) {
        std::vector<cv::Vec3d> rvecs_tmp, tvecs_tmp;
        std::vector<double> reproj_errors;
        cv::solvePnPGeneric(marker_obj_points_, marker_corners[i], camera_matrix, distortion_coeffs,
          rvecs_tmp, tvecs_tmp, false, cv::SOLVEPNP_IPPE_SQUARE, cv::noArray(), cv::noArray(),
          reproj_errors);

        if (rvecs_tmp.empty() || tvecs_tmp.empty()) {
          continue;
        }

        marker_poses[i].marker_id = marker_ids[i];
        marker_poses[i].pose = select_pose_from_candidates(
          rvecs_tmp, tvecs_tmp, reproj_errors, params_.pose_selector);
      }
  });
}

void ArucoDetector::estimate_board_poses(
  const std::vector<int> & marker_ids,
  const std::vector<std::vector<cv::Point2f>> & marker_corners,
  std::vector<BoardPoseOut> & board_poses,
  std::vector<cv::Vec3d> & rvecs,
  std::vector<cv::Vec3d> & tvecs) const
{
  cv::Mat camera_matrix, distortion_coeffs;
  {
    std::lock_guard<std::mutex> lk(intrinsics_mutex_);
    camera_matrix_.copyTo(camera_matrix);
    distortion_coeffs_.copyTo(distortion_coeffs);
  }

  for (const auto & board_desc : boards_) {
    const std::string name = board_desc.first;
    auto & board = board_desc.second;

    cv::Vec3d rvec, tvec;
    int valid = cv::aruco::estimatePoseBoard(marker_corners, marker_ids, board,
                                             camera_matrix, distortion_coeffs, rvec, tvec);
    if (valid > 0) {
      BoardPoseOut bpose;
      bpose.board_name = name;
      bpose.pose = convert_rvec_tvec(rvec, tvec);
      board_poses.push_back(bpose);
      rvecs.push_back(rvec);
      tvecs.push_back(tvec);
    }
  }
}

}  // namespace aruco_opencv
