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

#include <opencv2/calib3d.hpp>

namespace aruco_opencv
{

ArucoDetector::ArucoDetector()
: camera_matrix_(3, 3, CV_64FC1),
  distortion_coeffs_(4, 1, CV_64FC1, cv::Scalar(0)),
  marker_obj_points_(4, 1, CV_32FC3)
{}

void ArucoDetector::setDictionary(const cv::Ptr<cv::aruco::Dictionary> & dict)
{
  dictionary_ = dict;
}

void ArucoDetector::setDetectorParameters(const cv::Ptr<cv::aruco::DetectorParameters> & params)
{
  detector_parameters_ = params;
}

void ArucoDetector::setMarkerSize(double marker_size)
{
  marker_obj_points_.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-marker_size / 2.f, marker_size / 2.f, 0);
  marker_obj_points_.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(marker_size / 2.f, marker_size / 2.f, 0);
  marker_obj_points_.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(marker_size / 2.f, -marker_size / 2.f, 0);
  marker_obj_points_.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-marker_size / 2.f, -marker_size / 2.f, 0);
}

void ArucoDetector::setCameraIntrinsics(const cv::Mat & camera_matrix, const cv::Mat & dist_coeffs)
{
  std::lock_guard<std::mutex> lk(intrinsics_mutex_);
  camera_matrix.copyTo(camera_matrix_);
  dist_coeffs.copyTo(distortion_coeffs_);
}

void ArucoDetector::setBoards(
  const std::vector<std::pair<std::string,
  cv::Ptr<cv::aruco::Board>>> & boards)
{
  boards_ = boards;
}

void ArucoDetector::detect(
  const cv::Mat & image,
  std::vector<int> & marker_ids,
  std::vector<std::vector<cv::Point2f>> & marker_corners) const
{
  cv::aruco::detectMarkers(image, dictionary_, marker_corners, marker_ids, detector_parameters_);
}

void ArucoDetector::estimateMarkerPoses(
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
        cv::solvePnP(marker_obj_points_, marker_corners[i], camera_matrix, distortion_coeffs,
                   rvecs[i], tvecs[i], false, cv::SOLVEPNP_IPPE_SQUARE);
        marker_poses[i].marker_id = marker_ids[i];
        marker_poses[i].pose = convert_rvec_tvec(rvecs[i], tvecs[i]);
      }
  });
}

void ArucoDetector::estimateBoardPoses(
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
