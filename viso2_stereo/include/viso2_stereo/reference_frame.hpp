#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <libviso2/viso_stereo.h>

namespace viso2_stereo
{
namespace reference_frame
{
typedef enum
{
  Always = 0,
  MotionThreshold,
  InlierThreshold
} ChangeMethod;
}

class ReferenceFrame
{
private:
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_;
  reference_frame::ChangeMethod change_method_;
  double motion_threshold_; // method 1. Change the reference frame if last motion is small
  int inlier_threshold_; // method 2. Change the reference frame if the number of inliers is low

public:
  typedef std::shared_ptr<ReferenceFrame> SharedPtr;

  ReferenceFrame(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr params,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging) : logging_(logging)
  {
    change_method_ = (reference_frame::ChangeMethod)params->declare_parameter(
      "ref_frame.change_method", rclcpp::ParameterValue(1)).get<int>();
    motion_threshold_ = params->declare_parameter(
      "ref_frame.motion_threshold", rclcpp::ParameterValue(5.0)).get<double>();
    inlier_threshold_ = params->declare_parameter(
      "ref_frame.inlier_threshold", rclcpp::ParameterValue(150)).get<int>();
  }

  static double computeFeatureFlow(
    const std::vector<Matcher::p_match> & matches)
  {
    double total_flow = 0.0;
    for (size_t i = 0; i < matches.size(); ++i) {
      double x_diff = matches[i].u1c - matches[i].u1p;
      double y_diff = matches[i].v1c - matches[i].v1p;
      total_flow += sqrt(x_diff * x_diff + y_diff * y_diff);
    }
    return total_flow / matches.size();
  }

  bool update(std::shared_ptr<VisualOdometryStereo> odometer, bool change_before)
  {
    switch (change_method_) {
      case reference_frame::MotionThreshold:
        {
          double feature_flow = computeFeatureFlow(odometer->getMatches());
          auto change = (feature_flow < motion_threshold_);
          RCLCPP_DEBUG_STREAM(
            logging_->get_logger(),
            "Feature flow is " << feature_flow <<
              ", marking last motion as " <<
            (change_before ? "small." : "normal."));
          return change;
        }
      case reference_frame::InlierThreshold:
        {
          return
            odometer->getNumberOfInliers() > inlier_threshold_;
        }
      default:
        return false;
    }
  }
};
}
