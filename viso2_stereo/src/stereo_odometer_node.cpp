#include <libviso2/viso_stereo.h>
#include <image_geometry/stereo_camera_model.h>
#include <array>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <viso2_stereo/msg/viso_info.hpp>

#include "viso2_stereo/covariance.hpp"
#include "viso2_stereo/reference_frame.hpp"
#include "viso2_stereo/stereo_subscriber.hpp"
#include "viso2_stereo/odometry_publisher.hpp"
#include "viso2_stereo/odometry_params.hpp"

namespace viso2_stereo
{

using viso2_stereo::msg::VisoInfo;
using diagnostic_msgs::msg::DiagnosticStatus;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::CameraInfo;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;

tf2::Transform to_ros(Matrix matrix)
{
  tf2::Matrix3x3 rot_mat(
    matrix.val[0][0], matrix.val[0][1], matrix.val[0][2],
    matrix.val[1][0], matrix.val[1][1], matrix.val[1][2],
    matrix.val[2][0], matrix.val[2][1], matrix.val[2][2]);
  tf2::Vector3 t(matrix.val[0][3], matrix.val[1][3], matrix.val[2][3]);
  tf2::Transform delta_transform(rot_mat, t);
  return delta_transform;
}

class StereoOdometerNode : public rclcpp::Node
{

private:
  std::shared_ptr<VisualOdometryStereo> visual_odometer_;
  VisualOdometryStereo::parameters visual_odometer_params_;
  std::shared_ptr<StereoSubscriber> sub_;
  message_filters::Connection sub_connection_;
  std::shared_ptr<OdometryPublisher> pub_;
  CovarianceStrategy::SharedPtr covariance_;
  ReferenceFrame::SharedPtr reference_frame_;
  bool change_reference_frame_;

  rclcpp::Publisher<VisoInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<DiagnosticStatus>::SharedPtr diagnostic_pub_;

  bool got_lost_;
  Matrix reference_motion_;

  static constexpr std::array<double, 36> BAD_COVARIANCE =
  {{99999, 0, 0, 0, 0, 0,
    0, 99999, 0, 0, 0, 0,
    0, 0, 99999, 0, 0, 0,
    0, 0, 0, 99999, 0, 0,
    0, 0, 0, 0, 99999, 0,
    0, 0, 0, 0, 0, 99999}};

  // covariances
  std::array<double, 36> pose_covariance_;
  std::array<double, 36> twist_covariance_;

public:
  explicit StereoOdometerNode(const rclcpp::NodeOptions & options)
  : Node("stereo_odometer", options)
  {
    odometry_params::load_params(this->get_node_parameters_interface(), visual_odometer_params_);

    info_pub_ = this->create_publisher<VisoInfo>("info", 1);
    diagnostic_pub_ = this->create_publisher<DiagnosticStatus>("diagnostic", 1);
    reference_motion_ = Matrix::eye(4);

    reference_frame_ = std::make_shared<ReferenceFrame>(
      this->get_node_parameters_interface(), this->get_node_logging_interface());

    sub_ = std::make_shared<StereoSubscriber>(this);
    sub_connection_ = sub_->addCallback(
      std::bind(
        &StereoOdometerNode::imageCallback,
        this, _1, _2, _3, _4));

    // TODO(ZeilingerM) enable reset service

    pose_covariance_.fill(0.0);
    twist_covariance_.fill(0.0);
    CovarianceStrategyFactory factory;
    covariance_ = factory.create(this->get_node_parameters_interface());
  }

protected:
  void init_odometer(
    const CameraInfo::ConstSharedPtr l_info_msg,
    const CameraInfo::ConstSharedPtr r_info_msg)
  {
    if (l_info_msg->header.frame_id.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Left image must have a frame id");
      return;
    }
    image_geometry::StereoCameraModel model;
    model.fromCameraInfo(l_info_msg, r_info_msg);
    visual_odometer_params_.base = model.baseline();
    visual_odometer_params_.calib.cu = model.left().cx();
    visual_odometer_params_.calib.cv = model.left().cy();
    visual_odometer_params_.calib.f = model.left().fx();

    visual_odometer_.reset(new VisualOdometryStereo(visual_odometer_params_));
    pub_ = std::make_shared<OdometryPublisher>(
      this->get_node_clock_interface(),
      this->get_node_parameters_interface(),
      this->get_node_topics_interface(),
      this->get_node_logging_interface(), l_info_msg->header.frame_id);
  }

  void imageCallback(
    const Image::ConstSharedPtr & l_image_msg,
    const Image::ConstSharedPtr & r_image_msg,
    const CameraInfo::ConstSharedPtr & l_info_msg,
    const CameraInfo::ConstSharedPtr & r_info_msg)
  {
    if (l_image_msg->encoding != sensor_msgs::image_encodings::MONO8) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Left input image must have " << sensor_msgs::image_encodings::MONO8 << " encoding");
      return;
    }
    if (r_image_msg->encoding != sensor_msgs::image_encodings::MONO8) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Right input image must have " << sensor_msgs::image_encodings::MONO8 << " encoding");
      return;
    }

    bool first_run = false;
    // create odometer if not exists
    if (!visual_odometer_) {
      first_run = true;
      init_odometer(l_info_msg, r_info_msg);
    }

    // convert images if necessary
    const uint8_t * l_image_data;
    const uint8_t * r_image_data;
    int32_t l_step;
    l_image_data = l_image_msg->data.data();
    l_step = static_cast<int32_t>(l_image_msg->step);
    r_image_data = r_image_msg->data.data();

    auto start_time = this->now();
    int32_t dims[] =
    {static_cast<int32_t>(l_image_msg->width), static_cast<int32_t>(l_image_msg->height),
      l_step};

    tf2::Transform delta_transform;
    bool dont_publish = false;
    // on first run or when odometer got lost, only feed the odometer with
    // images without retrieving data
    if (first_run || got_lost_) {
      visual_odometer_->process(l_image_data, r_image_data, dims);
      got_lost_ = false;
      // on first run publish zero once
      if (first_run) {
        delta_transform.setIdentity();
      } else {
        dont_publish = true;
      }
    } else {
      bool success = visual_odometer_->process(
        l_image_data, r_image_data, dims, change_reference_frame_);
      if (success) {
        Matrix motion = Matrix::inv(visual_odometer_->getMotion());
        RCLCPP_DEBUG(
          this->get_logger(),
          "Found %i matches with %i inliers.",
          visual_odometer_->getNumberOfMatches(),
          visual_odometer_->getNumberOfInliers());
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Computed motion:\n" << motion);
        Matrix camera_motion;
        // if image was replaced due to small motion we have to subtract the
        // last motion to get the increment
        if (change_reference_frame_) {
          camera_motion = Matrix::inv(reference_motion_) * motion;
        } else {
          // image was not replaced, report full motion from odometer
          camera_motion = motion;
        }
        reference_motion_ = motion; // store last motion as reference

        std::vector<Matcher::p_match> matches = visual_odometer_->getMatches();
        std::vector<int> inlier_indices = visual_odometer_->getInlierIndices();
        delta_transform = to_ros(camera_motion);

        DiagnosticStatus ds;
        ds.hardware_id = this->get_name();
        ds.level = DiagnosticStatus::OK;
        ds.name = this->get_name();
        ds.message = "";
        ds.values.resize(1);
        ds.values[0].key = "d";
        ds.values[0].value = "0.0";
        diagnostic_pub_->publish(ds);
      } else {
        pose_covariance_ = BAD_COVARIANCE;
        twist_covariance_ = BAD_COVARIANCE;
        delta_transform.setIdentity();

        RCLCPP_WARN(this->get_logger(), "Visual odometer got lost");
        got_lost_ = true;
      }

      if (!got_lost_) {
        covariance_->compute(visual_odometer_, pose_covariance_, twist_covariance_);
      }

      if (!dont_publish) {
        pub_->update_and_publish(
          visual_odometer_, delta_transform, l_image_msg->header.stamp,
          pose_covariance_, twist_covariance_);
      }

      change_reference_frame_ = success ? reference_frame_->update(
        visual_odometer_,
        change_reference_frame_) : false;
      if (!change_reference_frame_) {
        RCLCPP_DEBUG(this->get_logger(), "Changing reference frame");
      }

      // create and publish viso2 info msg
      VisoInfo info_msg;
      info_msg.header.stamp = l_image_msg->header.stamp;
      info_msg.got_lost = !success;
      info_msg.change_reference_frame = !change_reference_frame_;
      info_msg.num_matches = visual_odometer_->getNumberOfMatches();
      info_msg.num_inliers = visual_odometer_->getNumberOfInliers();
      auto time_elapsed = this->now() - start_time;
      info_msg.runtime = time_elapsed.seconds();
      info_pub_->publish(info_msg);
    }
  }
};

} // namespace viso2_stereo

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(viso2_stereo::StereoOdometerNode)
