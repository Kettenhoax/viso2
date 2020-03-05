#pragma once

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <string>
#include <memory>
#include <utility>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/empty.hpp>
#include "viso2_stereo/covariance.hpp"

namespace viso2_stereo
{

using nav_msgs::msg::Odometry;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;
using rclcpp::callback_group::CallbackGroup;
using rclcpp::callback_group::CallbackGroupType;
using std::string;

class OdometryPublisher
{
public:
  typedef rclcpp::node_interfaces::NodeClockInterface ClockInterface;
  typedef rclcpp::node_interfaces::NodeParametersInterface ParamsInterface;
  typedef rclcpp::node_interfaces::NodeTopicsInterface TopicsInterface;
  typedef rclcpp::node_interfaces::NodeLoggingInterface LoggingInterface;

private:
  LoggingInterface::SharedPtr logging_;
  std::unique_ptr<CovarianceStrategy> covariance_;

  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr pose_pub_;

  // rclcpp::ServiceServer reset_service_;

  std::string sensor_frame_id_;
  std::string odom_frame_id_;
  std::string base_link_frame_id_;

  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  bool publish_tf_;
  bool invert_tf_;

  // the current integrated camera pose
  tf2::Transform integrated_pose_;
  // timestamp of the last update
  rclcpp::Time last_update_time_;
  // the latest motion of base to sensor <- added in order to avoid timing problems with the transform
  tf2::Stamped<tf2::Transform> base_to_sensor_;
  // indicates whether the transform from base to sensor has been set at least once
  bool base_to_sensor_set_;
  // enforces waiting for base <- sensor before publishing results
  bool wait_for_base_to_sensor_;
  // waits for correct velocities before publishing
  bool wait_for_velocities_;

  // initial pose of the base
  bool initial_base_pose_is_id_;
  bool initial_base_pose_set_;
  tf2::Stamped<tf2::Transform> initial_base_pose_;

public:
  OdometryPublisher(
    ClockInterface::SharedPtr clock_interface,
    ParamsInterface::SharedPtr params, TopicsInterface::SharedPtr topics,
    LoggingInterface::SharedPtr logging, std::string sensor_frame_id)
  : logging_(std::move(logging)), sensor_frame_id_(sensor_frame_id)
  {
    if (sensor_frame_id.empty()) {
      throw std::invalid_argument("sensor frame id must be defined");
    }

    buffer_ = std::make_shared<tf2_ros::Buffer>(clock_interface->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    // tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    odom_frame_id_ = params->declare_parameter(
      "odom_frame_id",
      rclcpp::ParameterValue("odom")).get<string>();
    base_link_frame_id_ =
      params->declare_parameter(
      "base_link_frame_id",
      rclcpp::ParameterValue("base_link")).get<string>();
    sensor_frame_id_ =
      params->declare_parameter(
      "sensor_frame_id",
      rclcpp::ParameterValue("camera")).get<string>();
    publish_tf_ = params->declare_parameter("publish_tf", rclcpp::ParameterValue(true)).get<bool>();
    wait_for_base_to_sensor_ = params->declare_parameter(
      "wait_for_base_to_sensor", rclcpp::ParameterValue(
        false)).get<bool>();
    wait_for_velocities_ =
      params->declare_parameter("wait_for_velocities", rclcpp::ParameterValue(false)).get<bool>();
    initial_base_pose_is_id_ = params->declare_parameter(
      "initialize_pose_as_id", rclcpp::ParameterValue(
        true)).get<bool>();

    rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options =
      rclcpp::PublisherOptions();
    callback_group_ = std::make_shared<CallbackGroup>(CallbackGroupType::Reentrant);

    auto publisher_factory = rclcpp::create_publisher_factory<Odometry, std::allocator<void>,
        rclcpp::Publisher<Odometry>>(options);
    odom_pub_ =
      std::dynamic_pointer_cast<rclcpp::Publisher<Odometry>>(
      topics->create_publisher(
        "odometry",
        publisher_factory, 2));
    topics->add_publisher(odom_pub_, callback_group_);

    auto pose_publisher_factory = rclcpp::create_publisher_factory<PoseStamped,
        std::allocator<void>,
        rclcpp::Publisher<PoseStamped>>(options);
    pose_pub_ =
      std::dynamic_pointer_cast<rclcpp::Publisher<PoseStamped>>(
      topics->create_publisher(
        "pose",
        pose_publisher_factory, 2));
    topics->add_publisher(pose_pub_, callback_group_);

    integrated_pose_.setIdentity();
    base_to_sensor_.setIdentity();
    initial_base_pose_.setIdentity();
    base_to_sensor_set_ = false;
    initial_base_pose_set_ = false;
  }

public:
  void update_and_publish(
    std::shared_ptr<VisualOdometryStereo>,
    const tf2::Transform & delta_transform, const rclcpp::Time & t, std::array<double,
    36> pose_covariance, std::array<double, 36> twist_covariance)
  {
    if (t < last_update_time_) {
      //TODO(ZeilingerM) handle at top level
      RCLCPP_WARN(
        logging_->get_logger(),
        "Saw negative time change in incoming sensor data, resetting pose.");
      integrated_pose_.setIdentity();
      buffer_->clear();
    }

    // integrate the pose
    integrated_pose_ *= delta_transform;

    // Try to get the transform from sensor to base
    std::string error_msg;
    try {
      auto baseToSensor = buffer_->lookupTransform(base_link_frame_id_, "source", t);
      tf2::fromMsg(base_to_sensor_, baseToSensor);
      base_to_sensor_set_ = true;
    } catch (tf2::TransformException & e) {
      if (!base_to_sensor_set_) {
        RCLCPP_WARN(
          logging_->get_logger(), "The tf from '%s' to '%s' does not seem to be available, "
          "last one will be used!",
          base_link_frame_id_.c_str(),
          sensor_frame_id_.c_str());
        RCLCPP_DEBUG(logging_->get_logger(), "Transform error: %s", error_msg.c_str());
      }
    }

    if (buffer_->canTransform(
        base_link_frame_id_, sensor_frame_id_, t, rclcpp::Duration(0),
        &error_msg))
    {
      auto baseToSensor = buffer_->lookupTransform(
        base_link_frame_id_,
        sensor_frame_id_,
        t);
      tf2::fromMsg(base_to_sensor_, baseToSensor);
    }

    // initialize the pose of base_link in odom or leave it set to id
    if (!initial_base_pose_is_id_) {
      // Try to initialize trajectory with current odom <- base_link or odom <- base_link_init
      if (!initial_base_pose_set_) {
        std::string error_msg;

        if (buffer_->canTransform(
            odom_frame_id_, base_link_frame_id_, t, rclcpp::Duration(0),
            &error_msg))
        {
          auto initialial_base_pose = buffer_->lookupTransform(
            odom_frame_id_, base_link_frame_id_, t);
          tf2::fromMsg(initial_base_pose_, initialial_base_pose);

          // Set the actual integrated pose to the identity, so the result is initialzed with the tf looked up
          integrated_pose_.setIdentity();

          initial_base_pose_set_ = true;
          RCLCPP_INFO(
            logging_->get_logger(),
            "Tf %s to %s AVAILABLE -> INITIALIZED stereo odometer",
            base_link_frame_id_.c_str(), odom_frame_id_.c_str());
        } else {
          std::string base_link_init_frame_id = base_link_frame_id_ + "_init";
          if (buffer_->canTransform(
              odom_frame_id_, base_link_init_frame_id, t, rclcpp::Duration(0),
              &error_msg))
          {
            auto initialial_base_pose = buffer_->lookupTransform(
              odom_frame_id_, base_link_init_frame_id, t);
            tf2::convert(initialial_base_pose, initial_base_pose_);

            // Set the actual integrated pose to the identity, so the result is initialzed with the tf looked up
            integrated_pose_.setIdentity();

            initial_base_pose_set_ = true;
            RCLCPP_INFO(
              logging_->get_logger(),
              "Tf %s to %s AVAILABLE -> INITIALIZED stereo odometer",
              base_link_init_frame_id.c_str(), odom_frame_id_.c_str());
          } else {
            RCLCPP_WARN(
              logging_->get_logger(),
              "Tf %s (or %s) to %s NOT available -> Cannot initialize stereo odometer",
              base_link_frame_id_.c_str(), base_link_init_frame_id.c_str(),
              odom_frame_id_.c_str());
            return;
          }
        }
      }
    }

    // transform integrated pose to base frame
    tf2::Transform base_transform = initial_base_pose_ * base_to_sensor_ * integrated_pose_ *
      base_to_sensor_.inverse();

    // Also transform the covariances
    transformCovariance(base_to_sensor_, pose_covariance);
    transformCovariance(base_to_sensor_, twist_covariance);

    RCLCPP_DEBUG(
      logging_->get_logger(),
      "pose covariance (xyzrpy): %.3f %.3f %.3f %.3f %.3f %.3f",
      pose_covariance[0], pose_covariance[7], pose_covariance[14],
      pose_covariance[21], pose_covariance[28], pose_covariance[35]);

    Odometry odometry_msg;
    odometry_msg.header.stamp = t;
    odometry_msg.header.frame_id = odom_frame_id_;
    odometry_msg.child_frame_id = base_link_frame_id_;
    tf2::toMsg(base_transform, odometry_msg.pose.pose);

    // calculate twist (not possible for first run as no delta_t can be computed)
    tf2::Transform delta_base_transform = base_to_sensor_ * delta_transform *
      base_to_sensor_.inverse();
    if (last_update_time_ > rclcpp::Time(0)) {
      double delta_t = (t - last_update_time_).seconds();
      if (delta_t) {
        odometry_msg.twist.twist.linear.x = delta_base_transform.getOrigin().getX() / delta_t;
        odometry_msg.twist.twist.linear.y = delta_base_transform.getOrigin().getY() / delta_t;
        odometry_msg.twist.twist.linear.z = delta_base_transform.getOrigin().getZ() / delta_t;
        tf2::Quaternion delta_rot = delta_base_transform.getRotation();
        double angle = delta_rot.getAngle();
        tf2::Vector3 axis = delta_rot.getAxis();
        tf2::Vector3 angular_twist = axis * angle / delta_t;
        odometry_msg.twist.twist.angular.x = angular_twist.x();
        odometry_msg.twist.twist.angular.y = angular_twist.y();
        odometry_msg.twist.twist.angular.z = angular_twist.z();
      }
    }

    // Check if base <- sensor and or velocities are mandatory for publishing and only publish if available
    bool publish_result =
      (!wait_for_base_to_sensor_ || base_to_sensor_set_) &&
      (!wait_for_velocities_ || last_update_time_.nanoseconds() != 0);
    odometry_msg.pose.covariance = pose_covariance;
    odometry_msg.twist.covariance = twist_covariance;

    PoseStamped pose_msg;
    pose_msg.header.stamp = odometry_msg.header.stamp;
    pose_msg.header.frame_id = odometry_msg.header.frame_id;
    pose_msg.pose = odometry_msg.pose.pose;

    if (publish_result) {
      odom_pub_->publish(odometry_msg);
      pose_pub_->publish(pose_msg);
    }

    if (publish_tf_ && publish_result) {
      tf2::Stamped<tf2::Transform> out_tfs(base_transform, tf2_ros::fromRclcpp(t), odom_frame_id_);
      TransformStamped out_tfs_msg;
      tf2::convert(out_tfs, out_tfs_msg);
      out_tfs_msg.child_frame_id = base_link_frame_id_;
      tf_broadcaster_->sendTransform(out_tfs_msg);
    }

    last_update_time_ = t;
  }

  /**
   * \brief Transforms the covariance matrix of either the pose or the twist.
   * \param[in] tf Linear transformation that should be applied
   * \param[in,out] cov Covariance matrix that is transformed in place
   */
  void transformCovariance(
    const tf2::Transform & tf,
    std::array<double, 36> & cov)
  {
    tf2::Matrix3x3 covT(cov[0], cov[1], cov[2],
      cov[6], cov[7], cov[8],
      cov[12], cov[13], cov[14]);

    tf2::Matrix3x3 covR(cov[21], cov[22], cov[23],
      cov[27], cov[28], cov[29],
      cov[33], cov[34], cov[35]);

    covT = tf.getBasis() * covT * tf.getBasis().transpose();
    covR = tf.getBasis() * covR * tf.getBasis().transpose();

    for (int r = 0; r < 3; ++r) {
      cov[r * 6] = covT.getRow(r).x();
      cov[r * 6 + 1] = covT.getRow(r).y();
      cov[r * 6 + 2] = covT.getRow(r).z();

      cov[(r + 3) * 6 + 3] = covR.getRow(r).x();
      cov[(r + 3) * 6 + 4] = covR.getRow(r).y();
      cov[(r + 3) * 6 + 5] = covR.getRow(r).z();
    }
  }
};

} // namespace viso2_stereo
