#pragma once

#include <string>
#include <array>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "libviso2/viso_stereo.h"

namespace viso2_ros
{

class CovarianceStrategy
{
public:
  typedef std::array<double, 36> CovArray;
  typedef std::shared_ptr<CovarianceStrategy> SharedPtr;

  virtual void compute(
    std::shared_ptr<VisualOdometryStereo>, CovArray & pose_cov,
    CovArray & twist_cov) = 0;
};

void fill_diagonal(std::array<double, 36> & arr, std::array<double, 6> values)
{
  arr.fill(0.0);
  for (size_t i = 0; i < 6; i++) {
    arr[i + 6 * i] = values[i];
  }
}

class ConstantCovarianceStrategy : public CovarianceStrategy
{
private:
// some arbitrary values (0.1m^2 linear cov. 10deg^2. angular cov.)
  static constexpr std::array<double, 36> STANDARD_POSE_COVARIANCE =
  {{0.1, 0, 0, 0, 0, 0,
    0, 0.1, 0, 0, 0, 0,
    0, 0, 0.1, 0, 0, 0,
    0, 0, 0, 0.17, 0, 0,
    0, 0, 0, 0, 0.17, 0,
    0, 0, 0, 0, 0, 0.17}};
  static constexpr std::array<double, 36> STANDARD_TWIST_COVARIANCE =
  {{0.05, 0, 0, 0, 0, 0,
    0, 0.05, 0, 0, 0, 0,
    0, 0, 0.05, 0, 0, 0,
    0, 0, 0, 0.09, 0, 0,
    0, 0, 0, 0, 0.09, 0,
    0, 0, 0, 0, 0, 0.09}};

public:
  void compute(std::shared_ptr<VisualOdometryStereo>, CovArray & pose_cov, CovArray & twist_cov)
  {
    pose_cov = STANDARD_POSE_COVARIANCE;
    twist_cov = STANDARD_TWIST_COVARIANCE;
  }
};

typedef struct
{
  int nof_inliers_min; ///< Minimum number of inliers required
  int nof_inliers_ok; ///< Intermediate number of inliers
  int nof_inliers_good; ///< Number of inliers of high quality results

  double cov_pos_min; ///< Position covariance at minimum number of inliers required
  double cov_pos_ok; ///< Position covariance at intermediate number of inliers
  double cov_pos_good; ///< Position covariance at number of inliers of high quality results

  double cov_ori_min; ///< Orientation covariance at minimum number of inliers required
  double cov_ori_ok; ///< Orientation covariance at intermediate number of inliers
  double cov_ori_good; ///< Orientation covariance at number of inliers of high quality results
} InlierCovarianceOptions;

class InlierBasedCovarianceStrategy : public CovarianceStrategy
{
private:
  InlierCovarianceOptions opts_;

public:
  explicit InlierBasedCovarianceStrategy(InlierCovarianceOptions opts)
  : opts_(opts) {}

  void compute(
    std::shared_ptr<VisualOdometryStereo> odometer, CovArray & pose_cov,
    CovArray & twist_cov)
  {
    // Set covariance based on number of inliers
    double covPos = 999999.0, covOri = 999999.0;

    int nof_inliers = odometer->getNumberOfInliers();

    double w1 = 0.0, w2 = 0.0;

    if (nof_inliers > opts_.nof_inliers_min) {
      if (nof_inliers > opts_.nof_inliers_ok) {
        // Matching is good
        nof_inliers = nof_inliers > opts_.nof_inliers_good ? opts_.nof_inliers_good : nof_inliers;
        w2 = static_cast<double>(nof_inliers - opts_.nof_inliers_ok) /
          static_cast<double>(opts_.nof_inliers_good - opts_.nof_inliers_ok);
        w1 = 1.0 - w2;
        covPos = w1 * opts_.cov_pos_ok + w2 * opts_.cov_pos_good;
        covOri = w1 * opts_.cov_ori_ok + w2 * opts_.cov_ori_good;
      } else {
        // Matching is useable
        w2 = static_cast<double>(nof_inliers - opts_.nof_inliers_min) /
          static_cast<double>(opts_.nof_inliers_ok - opts_.nof_inliers_min);
        w1 = 1.0 - w2;
        covPos = w1 * opts_.cov_pos_min + w2 * opts_.cov_pos_ok;
        covOri = w1 * opts_.cov_ori_min + w2 * opts_.cov_ori_ok;
      }
    }

    // ROS_INFO("nof inliers: %d  pos: %.3f  ori: %.3f", nof_inliers, covPos, covOri);
    fill_diagonal(pose_cov, {covPos, covPos, covPos, covOri, covOri, covOri});
    fill_diagonal(twist_cov, {covPos, covPos, covPos, covOri, covOri, covOri});
  }
};

typedef struct
{

} SVDCovarianceOptions;

class SVDCovarianceStrategy : public CovarianceStrategy
{
private:
  SVDCovarianceOptions opts_;

public:
  explicit SVDCovarianceStrategy(SVDCovarianceOptions opts)
  : opts_(opts) {}

  void compute(
    std::shared_ptr<VisualOdometryStereo> odometer, CovArray & pose_cov,
    CovArray & twist_cov)
  {
    std::array<double, 6> diagonal;

    for (size_t i = 0; i < 6; i++) {
      diagonal[i] = odometer->getCovariance()[(3 + i) % 6];
    }

    fill_diagonal(pose_cov, diagonal);
    fill_diagonal(twist_cov, diagonal);
    // ROS_DEBUG(
    //   "cov in camera: %.3f %.3f %.3f %.3f %.3f %.3f",
    //   pose_covariance[0], pose_covariance[7], pose_covariance[14],
    //   pose_covariance[21], pose_covariance[28], pose_covariance[35]);
  }
};

class CovarianceStrategyFactory
{
public:
  typedef rclcpp::node_interfaces::NodeParametersInterface ParamsInterface;

  CovarianceStrategy::SharedPtr create(ParamsInterface::SharedPtr params)
  {
    auto mode = params->declare_parameter(
      "cov_mode",
      rclcpp::ParameterValue("Standard")).get<std::string>();
    if (mode == "Standard") {
      return std::make_unique<ConstantCovarianceStrategy>();
    } else if (mode == "InlierBased") {
      InlierCovarianceOptions opts;
      opts.nof_inliers_min = params->declare_parameter("nof_inliers_min").get<double>();
      opts.nof_inliers_ok = params->declare_parameter("nof_inliers_ok").get<double>();
      opts.nof_inliers_good = params->declare_parameter("nof_inliers_good").get<double>();

      opts.cov_pos_min = params->declare_parameter("cov_pos_min").get<double>();
      opts.cov_pos_ok = params->declare_parameter("cov_pos_ok").get<double>();
      opts.cov_pos_good = params->declare_parameter("cov_pos_good").get<double>();

      opts.cov_ori_min = params->declare_parameter("cov_ori_min").get<double>();
      opts.cov_ori_ok = params->declare_parameter("cov_ori_ok").get<double>();
      opts.cov_ori_good = params->declare_parameter("cov_ori_good").get<double>();

      return std::make_unique<InlierBasedCovarianceStrategy>(opts);
    } else if (mode == "SVD") {
      SVDCovarianceOptions opts;
      return std::make_unique<SVDCovarianceStrategy>(opts);
    }
    throw std::invalid_argument("Unknown covariance strategy '" + mode + "'");
  }
};

} // namespace viso2_ros
