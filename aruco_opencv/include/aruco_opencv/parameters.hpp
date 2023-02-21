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

#pragma once

#include <map>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace aruco_opencv
{

template<class NodeT, typename T>
inline void declare_param(
  NodeT && node, std::string param_name,
  T default_value, bool dynamic = false)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = !dynamic;

  node.declare_parameter(param_name, rclcpp::ParameterValue(default_value), descriptor);
}

template<class NodeT, typename T>
inline void get_param(
  NodeT && node, std::string param_name, T & out_value, std::string log_info = "")
{
  node.get_parameter(param_name, out_value);

  if (!log_info.empty()) {
    RCLCPP_INFO_STREAM(node.get_logger(), log_info << out_value);
  }
}


template<class NodeT>
inline void declare_param_int_range(
  NodeT && node, std::string param_name,
  int default_value, int min_value, int max_value)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;

  auto range = rcl_interfaces::msg::IntegerRange();
  range.from_value = min_value;
  range.to_value = max_value;

  descriptor.integer_range.push_back(range);

  node.declare_parameter(param_name, default_value, descriptor);
}

template<class NodeT>
inline void declare_param_double_range(
  NodeT && node, std::string param_name,
  double default_value, double min_value, double max_value)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;

  auto range = rcl_interfaces::msg::FloatingPointRange();
  range.from_value = min_value;
  range.to_value = max_value;

  descriptor.floating_point_range.push_back(range);

  node.declare_parameter(param_name, default_value, descriptor);
}

template<class NodeT>
inline void declare_aruco_parameters(NodeT && node)
{
  declare_param_int_range(node, "aruco.adaptiveThreshWinSizeMin", 3, 3, 100);
  declare_param_int_range(node, "aruco.adaptiveThreshWinSizeMax", 23, 3, 100);
  declare_param_int_range(node, "aruco.adaptiveThreshWinSizeStep", 10, 1, 100);
  declare_param_double_range(node, "aruco.adaptiveThreshConstant", 7.0, 0.0, 100.0);
  declare_param_double_range(node, "aruco.minMarkerPerimeterRate", 0.03, 0.0, 4.0);
  declare_param_double_range(node, "aruco.maxMarkerPerimeterRate", 4.0, 0.0, 4.0);
  declare_param_double_range(node, "aruco.polygonalApproxAccuracyRate", 0.03, 0.0, 0.3);
  declare_param_double_range(node, "aruco.minCornerDistanceRate", 0.05, 0.0, 0.25);
  declare_param_int_range(node, "aruco.minDistanceToBorder", 3, 0, 100);
  declare_param_double_range(node, "aruco.minMarkerDistanceRate", 0.05, 0.0, 0.25);
  declare_param_int_range(node, "aruco.markerBorderBits", 1, 1, 3);
  declare_param_int_range(node, "aruco.perspectiveRemovePixelPerCell", 4, 1, 20);
  declare_param_double_range(node, "aruco.perspectiveRemoveIgnoredMarginPerCell", 0.13, 0.0, 0.5);
  declare_param_double_range(node, "aruco.maxErroneousBitsInBorderRate", 0.35, 0.0, 1.0);
  declare_param_double_range(node, "aruco.minOtsuStdDev", 5.0, 0.0, 30.0);
  declare_param_double_range(node, "aruco.errorCorrectionRate", 0.6, 0.0, 1.0);
  declare_param_int_range(node, "aruco.cornerRefinementMethod", 2, 0, 2);
  declare_param_int_range(node, "aruco.cornerRefinementWinSize", 5, 2, 10);
  declare_param_int_range(node, "aruco.cornerRefinementMaxIterations", 30, 1, 100);
  declare_param_double_range(node, "aruco.cornerRefinementMinAccuracy", 0.1, 0.01, 1.0);
}

template<class NodeT>
void retrieve_aruco_parameters(
  NodeT && node, cv::Ptr<cv::aruco::DetectorParameters> & detector_parameters,
  bool log_values = false)
{
  node.get_parameter(
    "aruco.adaptiveThreshWinSizeMin", detector_parameters->adaptiveThreshWinSizeMin);
  node.get_parameter(
    "aruco.adaptiveThreshWinSizeMax", detector_parameters->adaptiveThreshWinSizeMax);
  node.get_parameter(
    "aruco.adaptiveThreshWinSizeStep", detector_parameters->adaptiveThreshWinSizeStep);
  node.get_parameter(
    "aruco.adaptiveThreshConstant", detector_parameters->adaptiveThreshConstant);
  node.get_parameter(
    "aruco.minMarkerPerimeterRate", detector_parameters->minMarkerPerimeterRate);
  node.get_parameter(
    "aruco.maxMarkerPerimeterRate", detector_parameters->maxMarkerPerimeterRate);
  node.get_parameter(
    "aruco.polygonalApproxAccuracyRate", detector_parameters->polygonalApproxAccuracyRate);
  node.get_parameter(
    "aruco.minCornerDistanceRate", detector_parameters->minCornerDistanceRate);
  node.get_parameter(
    "aruco.minDistanceToBorder", detector_parameters->minDistanceToBorder);
  node.get_parameter(
    "aruco.minMarkerDistanceRate", detector_parameters->minMarkerDistanceRate);
  node.get_parameter(
    "aruco.markerBorderBits", detector_parameters->markerBorderBits);
  node.get_parameter(
    "aruco.perspectiveRemovePixelPerCell", detector_parameters->perspectiveRemovePixelPerCell);
  node.get_parameter(
    "aruco.perspectiveRemoveIgnoredMarginPerCell",
    detector_parameters->perspectiveRemoveIgnoredMarginPerCell);
  node.get_parameter(
    "aruco.maxErroneousBitsInBorderRate", detector_parameters->maxErroneousBitsInBorderRate);
  node.get_parameter(
    "aruco.minOtsuStdDev", detector_parameters->minOtsuStdDev);
  node.get_parameter(
    "aruco.errorCorrectionRate", detector_parameters->errorCorrectionRate);
  node.get_parameter(
    "aruco.cornerRefinementMethod", detector_parameters->cornerRefinementMethod);
  node.get_parameter(
    "aruco.cornerRefinementWinSize", detector_parameters->cornerRefinementWinSize);
  node.get_parameter(
    "aruco.cornerRefinementMaxIterations", detector_parameters->cornerRefinementMaxIterations);
  node.get_parameter(
    "aruco.cornerRefinementMinAccuracy", detector_parameters->cornerRefinementMinAccuracy);

  if (log_values) {
    RCLCPP_INFO_STREAM(
      node.get_logger(),
      " * adaptiveThreshWinSizeMin: " << detector_parameters->adaptiveThreshWinSizeMin);
    RCLCPP_INFO_STREAM(
      node.get_logger(),
      " * adaptiveThreshWinSizeMax: " << detector_parameters->adaptiveThreshWinSizeMax);
    RCLCPP_INFO_STREAM(
      node.get_logger(),
      " * adaptiveThreshWinSizeStep: " << detector_parameters->adaptiveThreshWinSizeStep);
    RCLCPP_INFO_STREAM(
      node.get_logger(),
      " * adaptiveThreshConstant: " << detector_parameters->adaptiveThreshConstant);
    RCLCPP_INFO_STREAM(
      node.get_logger(),
      " * minMarkerPerimeterRate: " << detector_parameters->minMarkerPerimeterRate);
    RCLCPP_INFO_STREAM(
      node.get_logger(),
      " * maxMarkerPerimeterRate: " << detector_parameters->maxMarkerPerimeterRate);
    RCLCPP_INFO_STREAM(
      node.get_logger(),
      " * polygonalApproxAccuracyRate: " << detector_parameters->polygonalApproxAccuracyRate);
    RCLCPP_INFO_STREAM(
      node.get_logger(),
      " * minCornerDistanceRate: " << detector_parameters->minCornerDistanceRate);
    RCLCPP_INFO_STREAM(
      node.get_logger(),
      " * minDistanceToBorder: " << detector_parameters->minDistanceToBorder);
    RCLCPP_INFO_STREAM(
      node.get_logger(),
      " * minMarkerDistanceRate: " << detector_parameters->minMarkerDistanceRate);
    RCLCPP_INFO_STREAM(
      node.get_logger(),
      " * markerBorderBits: " << detector_parameters->markerBorderBits);
    RCLCPP_INFO_STREAM(
      node.get_logger(),
      " * perspectiveRemovePixelPerCell: " << detector_parameters->perspectiveRemovePixelPerCell);
    RCLCPP_INFO_STREAM(
      node.get_logger(),
      " * perspectiveRemoveIgnoredMarginPerCell: " <<
        detector_parameters->perspectiveRemoveIgnoredMarginPerCell);
    RCLCPP_INFO_STREAM(
      node.get_logger(),
      " * maxErroneousBitsInBorderRate: " << detector_parameters->maxErroneousBitsInBorderRate);
    RCLCPP_INFO_STREAM(
      node.get_logger(),
      " * minOtsuStdDev: " << detector_parameters->minOtsuStdDev);
    RCLCPP_INFO_STREAM(
      node.get_logger(),
      " * errorCorrectionRate: " << detector_parameters->errorCorrectionRate);
    std::map<int, std::string> crmethod = {{0, "NONE"}, {1, "SUBPIX"}, {2, "CONTOUR"}};
    RCLCPP_INFO_STREAM(
      node.get_logger(),
      " * cornerRefinementMethod: " << detector_parameters->cornerRefinementMethod << " (" <<
        crmethod[detector_parameters->cornerRefinementMethod] << ")");
    RCLCPP_INFO_STREAM(
      node.get_logger(),
      " * cornerRefinementWinSize: " << detector_parameters->cornerRefinementWinSize);
    RCLCPP_INFO_STREAM(
      node.get_logger(),
      " * cornerRefinementMaxIterations: " << detector_parameters->cornerRefinementMaxIterations);
    RCLCPP_INFO_STREAM(
      node.get_logger(),
      " * cornerRefinementMinAccuracy: " << detector_parameters->cornerRefinementMinAccuracy);
  }
}

}  // namespace aruco_opencv
