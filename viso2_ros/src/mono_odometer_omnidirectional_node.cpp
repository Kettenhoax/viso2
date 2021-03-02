#include <array>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.h>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <libviso2/viso_mono_omnidirectional.h>
#include <viso2_ros/msg/viso_info.hpp>
#include "viso2_ros/odometry_publisher.hpp"
#include "viso2_ros/odometry_params.hpp"


namespace viso2_ros
{

using viso2_ros::msg::VisoInfo;
using diagnostic_msgs::msg::DiagnosticStatus;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::CameraInfo;

class MonoOdometerOmnidirectionalNode : public rclcpp::Node
{

private:

  std::shared_ptr<VisualOdometryMonoOmnidirectional> visual_odometer_;
  VisualOdometryMonoOmnidirectional::parameters visual_odometer_params_;
  std::shared_ptr<OdometryPublisher> pub_;

  image_transport::Subscriber camera_sub_;

  rclcpp::Publisher<VisoInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<DiagnosticStatus>::SharedPtr diagnostic_pub_;

  bool replace_;
  // covariances
  std::array<double, 36> pose_covariance_;
  std::array<double, 36> twist_covariance_;


public:

  explicit MonoOdometerOmnidirectionalNode(const rclcpp::NodeOptions & options)
  : Node("mono_odometer_omnidirectional", options)
  {

    replace_ = false;
    odometry_params::load_params(this->get_node_parameters_interface(), visual_odometer_params_);

    info_pub_ = this->create_publisher<VisoInfo>("info", 1);
    diagnostic_pub_ = this->create_publisher<DiagnosticStatus>("diagnostic", 1);

    std::string image_topic;
    image_topic =
    this->get_node_parameters_interface()->declare_parameter(
    "/mono_odometer/image",
    rclcpp::ParameterValue(image_topic)).get<std::string>();

    image_transport::ImageTransport it(shared_from_this());
    camera_sub_ = it.subscribe(image_topic, 1, &MonoOdometerOmnidirectionalNode::imageCallback, this);

    pose_covariance_.fill(0.0);
    twist_covariance_.fill(0.0);

  }

protected:

  void imageCallback(
    const Image::ConstSharedPtr & image_msg)  {
 
    bool first_run = false;
    // create odometer if not exists
    if (!visual_odometer_)
    {
      first_run = true;
      visual_odometer_.reset(new VisualOdometryMonoOmnidirectional(visual_odometer_params_));
      
      pub_ = std::make_shared<OdometryPublisher>(
      this->get_node_clock_interface(),
      this->get_node_parameters_interface(),
      this->get_node_topics_interface(),
      this->get_node_logging_interface(), image_msg->header.frame_id);
    }

    // convert image if necessary
    const uint8_t *image_data;
    int32_t step;

    if (image_msg->encoding == sensor_msgs::image_encodings::MONO8)
    {
      image_data = const_cast<uint8_t*>(&(image_msg->data[0]));
    }
    else
    {
      image_data = image_msg->data.data();
    }
    step = static_cast<int32_t>(image_msg->step);

    // run the odometer
    auto start_time = this->now();
    int32_t dims[] =
    {static_cast<int32_t>(image_msg->width), static_cast<int32_t>(image_msg->height),
      step};
    // on first run, only feed the odometer with first image pair without
    // retrieving data
    tf2::Transform delta_transform;
    if (first_run)
    {
      visual_odometer_->process(image_data, dims);
      delta_transform.setIdentity();
    }
    else
    {
      bool success = visual_odometer_->process(image_data, dims);
      if(success)
      {
        replace_ = false;
        Matrix camera_motion = Matrix::inv(visual_odometer_->getMotion());
        RCLCPP_DEBUG(
          this->get_logger(),
          "Found %i matches with %i inliers.",
          visual_odometer_->getNumberOfMatches(),
          visual_odometer_->getNumberOfInliers());
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Computed motion:\n" << camera_motion);
        
        std::vector<Matcher::p_match> matches = visual_odometer_->getMatches();
        std::vector<int> inlier_indices = visual_odometer_->getInlierIndices();
        delta_transform = to_ros(camera_motion);
      }
      else
      {
        RCLCPP_DEBUG(this->get_logger(), "Call to VisualOdometryMonoOmnidirectional::process() failed. Assuming motion too small.");
        replace_ = true;
        delta_transform.setIdentity();
      }

      pub_->update_and_publish(
          visual_odometer_, delta_transform, image_msg->header.stamp,
          pose_covariance_, twist_covariance_);

      // create and publish viso2 info msg
      VisoInfo info_msg;
      info_msg.header.stamp = image_msg->header.stamp;
      info_msg.got_lost = !success;
      info_msg.change_reference_frame = false;
      info_msg.num_matches = visual_odometer_->getNumberOfMatches();
      info_msg.num_inliers = visual_odometer_->getNumberOfInliers();
      auto time_elapsed = this->now() - start_time;
      info_msg.runtime = time_elapsed.seconds();
      info_pub_->publish(info_msg);
    }
  }
};

} // end of namespace


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(viso2_ros::MonoOdometerOmnidirectionalNode)