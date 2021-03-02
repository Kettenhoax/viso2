#pragma once

#include <rclcpp/rclcpp.hpp>

#include <libviso2/viso_stereo.h>
#include <libviso2/viso_mono.h>
#include <libviso2/viso_mono_omnidirectional.h>
#include <fstream>

namespace viso2_ros
{

namespace odometry_params
{

using rclcpp::node_interfaces::NodeParametersInterface;

/// loads matcher params
void load_params(NodeParametersInterface::SharedPtr params, Matcher::parameters & out)
{
  out.nms_n =
    params->declare_parameter("match.nms_n", rclcpp::ParameterValue(out.nms_n)).get<int>();
  out.nms_tau = params->declare_parameter(
    "match.nms_tau",
    rclcpp::ParameterValue(out.nms_tau)).get<int>();
  out.match_binsize =
    params->declare_parameter(
    "match.match_binsize",
    rclcpp::ParameterValue(out.match_binsize)).get<int>();
  out.match_radius =
    params->declare_parameter(
    "match.match_radius",
    rclcpp::ParameterValue(out.match_radius)).get<int>();
  out.match_disp_tolerance = params->declare_parameter(
    "match.match_disp_tolerance",
    rclcpp::ParameterValue(out.match_disp_tolerance)).get<int>();
  out.outlier_disp_tolerance = params->declare_parameter(
    "match.outlier_disp_tolerance",
    rclcpp::ParameterValue(out.outlier_disp_tolerance)).get<int>();
  out.outlier_flow_tolerance = params->declare_parameter(
    "match.outlier_flow_tolerance",
    rclcpp::ParameterValue(out.outlier_flow_tolerance)).get<int>();
  out.multi_stage =
    params->declare_parameter(
    "match.multi_stage",
    rclcpp::ParameterValue(out.multi_stage)).get<int>();
  out.half_resolution =
    params->declare_parameter(
    "match.half_resolution",
    rclcpp::ParameterValue(out.half_resolution)).get<int>();
  out.refinement =
    params->declare_parameter(
    "match.refinement",
    rclcpp::ParameterValue(out.refinement)).get<int>();
}

/// loads bucketing params
void load_params(NodeParametersInterface::SharedPtr params, VisualOdometry::bucketing & bucketing)
{
  bucketing.max_features =
    params->declare_parameter(
    "bucket.max_features",
    rclcpp::ParameterValue(bucketing.max_features)).get<int>();
  bucketing.bucket_width =
    params->declare_parameter(
    "bucket.bucket_width",
    rclcpp::ParameterValue(bucketing.bucket_width)).get<double>();
  bucketing.bucket_height = params->declare_parameter(
    "bucket.bucket_height",
    rclcpp::ParameterValue(bucketing.bucket_height)).get<double>();
}

/// loads common odometry params
void load_common_params(NodeParametersInterface::SharedPtr params, VisualOdometry::parameters & out)
{
  load_params(params, out.match);
  load_params(params, out.bucket);
}

/// loads common & stereo specific params
void load_params(NodeParametersInterface::SharedPtr params, VisualOdometryStereo::parameters & out)
{
  load_common_params(params, out);
  out.ransac_iters =
    params->declare_parameter(
    "ransac_iters",
    rclcpp::ParameterValue(out.ransac_iters)).get<int>();
  out.inlier_threshold =
    params->declare_parameter(
    "inlier_threshold",
    rclcpp::ParameterValue(out.inlier_threshold)).get<double>();
  out.reweighting =
    params->declare_parameter(
    "reweighting",
    rclcpp::ParameterValue(out.reweighting)).get<bool>();
  out.cov_svd_factor =
    params->declare_parameter(
    "cov_svd_factor",
    rclcpp::ParameterValue(out.cov_svd_factor)).get<double>();
}

/// loads common & mono specific params
void load_params(NodeParametersInterface::SharedPtr params, VisualOdometryMono::parameters & out)
{
  load_common_params(params, out);
  out.height =
    params->declare_parameter(
    "camera_height",
    rclcpp::ParameterValue(out.height)).get<double>();
  out.pitch =
    params->declare_parameter(
    "camera_pitch",
    rclcpp::ParameterValue(out.pitch)).get<double>();
  out.ransac_iters =
    params->declare_parameter(
    "ransac_iters",
    rclcpp::ParameterValue(out.ransac_iters)).get<int>();
  out.inlier_threshold =
    params->declare_parameter(
    "inlier_threshold",
    rclcpp::ParameterValue(out.inlier_threshold)).get<double>();
  out.motion_threshold =
    params->declare_parameter(
    "motion_threshold",
    rclcpp::ParameterValue(out.motion_threshold)).get<double>();

}

/// loads common & omnidirectional mono specific params
void load_params(NodeParametersInterface::SharedPtr params, VisualOdometryMonoOmnidirectional::parameters & out)
{
  load_common_params(params, out); 
  out.ransac_iters =
    params->declare_parameter(
    "ransac_iters",
    rclcpp::ParameterValue(out.ransac_iters)).get<int>();
  out.inlier_threshold =
    params->declare_parameter(
    "inlier_threshold",
    rclcpp::ParameterValue(out.inlier_threshold)).get<double>();
  out.motion_threshold =
    params->declare_parameter(
    "motion_threshold",
    rclcpp::ParameterValue(out.motion_threshold)).get<double>();
  std::string path;
  path =
    params->declare_parameter(
    "calib_path",
    rclcpp::ParameterValue(path)).get<std::string>();
  std::ifstream file(path.c_str());
    if(file.is_open())
  {
    std::string s;

    //Read polynomial coefficients
    std::getline(file, s);
    file >> out.omnidirectional_calib.length_pol;
    for (int i = 0; i < out.omnidirectional_calib.length_pol; i++)
    {
      file >> out.omnidirectional_calib.pol[i];
    }

    //Read inverse polynomial coefficients
    std::getline(file, s);
    std::getline(file, s);
    std::getline(file, s);
    file >> out.omnidirectional_calib.length_invpol;
    for (int i = 0; i < out.omnidirectional_calib.length_invpol; i++)
    {
      file >> out.omnidirectional_calib.invpol[i];
    }

    //Read center coordinates
    std::getline(file, s);
    std::getline(file, s);
    std::getline(file, s);
    file >> out.omnidirectional_calib.xc >> 
            out.omnidirectional_calib.yc;
    
    //Read affine coefficients
    std::getline(file, s);
    std::getline(file, s);
    std::getline(file, s);
    file >> out.omnidirectional_calib.c >> 
            out.omnidirectional_calib.d >>
            out.omnidirectional_calib.e;
    
    //Read image size
    std::getline(file, s);
    std::getline(file, s);
    std::getline(file, s);
    file >> out.omnidirectional_calib.height >> 
            out.omnidirectional_calib.width;

    file.close();
  }
  else
  {
    std::cout << "File not found or path not provided" << std::endl;
    std::cout << "Using default parameters" << std::endl;
    out.omnidirectional_calib.fx = 1;
    out.omnidirectional_calib.fy = 1;
    out.omnidirectional_calib.cx = 0;
    out.omnidirectional_calib.cy = 0;
    out.omnidirectional_calib.xc = 1;
    out.omnidirectional_calib.yc = 1;
    out.omnidirectional_calib.c  = 1;
    out.omnidirectional_calib.d  = 1;
    out.omnidirectional_calib.e  = 1;

    out.omnidirectional_calib.length_pol    = 1;
    out.omnidirectional_calib.length_invpol = 1;
    out.omnidirectional_calib.width         = 2;
    out.omnidirectional_calib.height        = 2;

    out.omnidirectional_calib.pol[0]    = 1;
    out.omnidirectional_calib.invpol[0] = 1;
  }
}

} // end of namespace

std::ostream & operator<<(std::ostream & out, const Matcher::parameters & params)
{
  out << "  nms_n                  = " << params.nms_n << std::endl;
  out << "  nms_tau                = " << params.nms_tau << std::endl;
  out << "  match_binsize          = " << params.match_binsize << std::endl;
  out << "  match_radius           = " << params.match_radius << std::endl;
  out << "  match_disp_tolerance   = " << params.match_disp_tolerance << std::endl;
  out << "  outlier_disp_tolerance = " << params.outlier_disp_tolerance << std::endl;
  out << "  outlier_flow_tolerance = " << params.outlier_flow_tolerance << std::endl;
  out << "  multi_stage            = " << params.multi_stage << std::endl;
  out << "  half_resolution        = " << params.half_resolution << std::endl;
  out << "  refinement             = " << params.refinement << std::endl;
  return out;
}

std::ostream & operator<<(std::ostream & out, const VisualOdometry::calibration & calibration)
{
  out << "  f  = " << calibration.f << std::endl;
  out << "  cu = " << calibration.cu << std::endl;
  out << "  cv = " << calibration.cv << std::endl;
  return out;
}

std::ostream& operator<<(std::ostream& out, const VisualOdometry::omnidirectional_calibration& params)
{
  out << "  poly         = ";
  for (int i = 0; i < params.length_pol; i++) {
    out << params.pol[i] << " ; ";
  }
  out << std::endl;

  out << "  inverse poly = ";
  for (int i = 0; i < params.length_invpol; i++) {
    out << params.invpol[i] << " ; ";
  }
  out << std::endl;

  out << "  xc           = " << params.xc << std::endl;
  out << "  yc           = " << params.yc << std::endl;
  out << "  width        = " << params.width << std::endl;
  out << "  height       = " << params.height << std::endl;
  out << "  fx           = " << params.fx << std::endl;
  out << "  fy           = " << params.fy << std::endl;
  out << "  cx           = " << params.cx << std::endl;
  out << "  cy           = " << params.cy << std::endl;

  return out;
}

std::ostream & operator<<(std::ostream & out, const VisualOdometry::bucketing & bucketing)
{
  out << "  max_features  = " << bucketing.max_features << std::endl;
  out << "  bucket_width  = " << bucketing.bucket_width << std::endl;
  out << "  bucket_height = " << bucketing.bucket_height << std::endl;
  return out;
}

std::ostream & operator<<(std::ostream & out, const VisualOdometry::parameters & params)
{
  out << "Calibration parameters:" << std::endl << params.calib;
  out << "Matcher parameters:" << std::endl << params.match;
  out << "Bucketing parameters:" << std::endl << params.bucket;
  return out;
}

std::ostream & operator<<(std::ostream & out, const VisualOdometryStereo::parameters & params)
{
  out << static_cast<VisualOdometry::parameters>(params);
  out << "Stereo odometry parameters:" << std::endl;
  out << "  base             = " << params.base << std::endl;
  out << "  ransac_iters     = " << params.ransac_iters << std::endl;
  out << "  inlier_threshold = " << params.inlier_threshold << std::endl;
  out << "  reweighting      = " << params.reweighting << std::endl;
  return out;
}

std::ostream & operator<<(std::ostream & out, const VisualOdometryMono::parameters & params)
{
  out << static_cast<VisualOdometry::parameters>(params);
  out << "Mono odometry parameters:" << std::endl;
  out << "  camera_height    = " << params.height << std::endl;
  out << "  camera_pitch     = " << params.pitch << std::endl;
  out << "  ransac_iters     = " << params.ransac_iters << std::endl;
  out << "  inlier_threshold = " << params.inlier_threshold << std::endl;
  out << "  motion_threshold = " << params.motion_threshold << std::endl;
  return out;
}

} // namespace viso2_ros
