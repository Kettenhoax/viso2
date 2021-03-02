#pragma once

#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "message_filters/connection.h"
#include "message_filters/signal9.h"
#include <image_transport/subscriber_filter.h>

namespace viso2_ros
{

using namespace std::chrono_literals; // NOLINT
using namespace std::placeholders; // NOLINT

typedef struct
{
  size_t left_image = 0;
  size_t left_info = 0;
  size_t right_image = 0;
  size_t right_info = 0;
} StereoRecvCounter;

using sensor_msgs::msg::Image;
using sensor_msgs::msg::CameraInfo;
using image_transport::SubscriberFilter;
using message_filters::NullType;
using message_filters::Signal9;
using message_filters::Connection;
using message_filters::Subscriber;
using message_filters::sync_policies::ExactTime;
using message_filters::sync_policies::ApproximateTime;

/**
 * Subscription helper to stereo messages, providing diagnostics on the connection
 */
class StereoSubscriber
{

private:
  rclcpp::Node * node_;
  std::shared_ptr<SubscriberFilter> left_sub_, right_sub_;
  std::shared_ptr<Subscriber<CameraInfo>> left_info_sub_, right_info_sub_;
  typedef ExactTime<Image, Image, CameraInfo, CameraInfo> ExactPolicy;
  typedef ApproximateTime<Image, Image, CameraInfo, CameraInfo> ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  std::shared_ptr<ExactSync> exact_sync_;
  std::shared_ptr<ApproximateSync> approximate_sync_;

  size_t queue_size_;
  size_t all_received_;
  StereoRecvCounter recv_counter_;

  rclcpp::TimerBase::SharedPtr check_synced_timer_;
  static void increment(size_t * value)
  {
    ++(*value);
  }

  typedef Signal9<Image, Image, CameraInfo, CameraInfo, NullType, NullType, NullType, NullType,
      NullType> Signal;
  Signal signal_;

  void dataCb(
    const Image::ConstSharedPtr & l_image,
    const Image::ConstSharedPtr & r_image,
    const CameraInfo::ConstSharedPtr & l_info,
    const CameraInfo::ConstSharedPtr & r_info)
  {
    if (l_image->width != r_image->width) {
      RCLCPP_ERROR(node_->get_logger(), "Stereo images do not have same width");
    }
    if (l_image->height != r_image->height) {
      RCLCPP_ERROR(node_->get_logger(), "Stereo images do not have same height");
    }
    if (l_image->step != r_image->step) {
      RCLCPP_ERROR(node_->get_logger(), "Stereo images do not have same step");
    }
    if (l_image->encoding != r_image->encoding) {
      RCLCPP_ERROR(node_->get_logger(), "Stereo images do not have same encoding");
    }
    ++all_received_;
    message_filters::MessageEvent<Image> l_image_event(l_image);
    message_filters::MessageEvent<Image> r_image_event(r_image);
    message_filters::MessageEvent<CameraInfo> l_info_event(l_info);
    message_filters::MessageEvent<CameraInfo> r_info_event(r_info);
    message_filters::MessageEvent<NullType> empty;
    signal_.call(
      l_image_event, r_image_event, l_info_event, r_info_event, empty, empty, empty,
      empty, empty);
  }

  void check_input_synchronization()
  {
    size_t threshold = 3 * all_received_;
    if (recv_counter_.left_image >= threshold || recv_counter_.right_image >= threshold ||
      recv_counter_.left_info >= threshold || recv_counter_.right_info >= threshold)
    {
      RCLCPP_WARN(
        node_->get_logger(),
        "Low number of synchronized image/camera_info tuples received.\n"
        "  Left images:       %lu (topic '%s')\n"
        "  Right images:      %lu (topic '%s')\n"
        "  Left camera info:  %lu (topic '%s')\n"
        "  Right camera info: %lu (topic '%s')\n"
        "  Synchronized tuples: %lu\n"
        "Possible issues:\n"
        "  * Image processor providing image_rect is not running.\n"
        "    Does `ros2 node info %s` show any connections?\n"
        "  * The cameras are not synchronized.\n"
        "    Try setting the parameter 'approximate_sync' to true\n"
        "  * One or more messages of a tuple are dropped on the network.\n"
        "    Try to increase the network buffer size, or increase the queue size (currently %lu).",
        recv_counter_.left_image,
        left_sub_->getTopic().c_str(),
        recv_counter_.right_image, right_sub_->getTopic().c_str(),
        recv_counter_.left_info, left_info_sub_->getTopic().c_str(),
        recv_counter_.right_info, right_info_sub_->getTopic().c_str(),
        all_received_, node_->get_name(), queue_size_);
    }
  }

public:
  explicit StereoSubscriber(rclcpp::Node * node)
  : node_(node), all_received_(0)
  {
    image_transport::TransportHints hints(node, "raw");
    left_sub_ = std::make_shared<SubscriberFilter>(node, "left/image_rect", hints.getTransport());
    right_sub_ = std::make_shared<SubscriberFilter>(node, "right/image_rect", hints.getTransport());
    left_info_sub_ = std::make_shared<Subscriber<CameraInfo>>(
      node, "left/image_rect/camera_info",
      rclcpp::SensorDataQoS().get_rmw_qos_profile());
    right_info_sub_ = std::make_shared<Subscriber<CameraInfo>>(
      node, "right/image_rect/camera_info",
      rclcpp::SensorDataQoS().get_rmw_qos_profile());

    left_sub_->registerCallback(std::bind(&StereoSubscriber::increment, &recv_counter_.left_image));
    right_sub_->registerCallback(
      std::bind(
        &StereoSubscriber::increment,
        &recv_counter_.right_image));
    left_info_sub_->registerCallback(
      std::bind(
        &StereoSubscriber::increment,
        &recv_counter_.left_info));
    right_info_sub_->registerCallback(
      std::bind(
        &StereoSubscriber::increment,
        &recv_counter_.right_info));
    check_synced_timer_ =
      node->create_wall_timer(5s, std::bind(&StereoSubscriber::check_input_synchronization, this));

    queue_size_ = node->declare_parameter("queue_size", 5);
    auto approximate_sync = node->declare_parameter("approximate_sync", false);

    auto cb = std::bind(&StereoSubscriber::dataCb, this, _1, _2, _3, _4);
    if (approximate_sync) {
      approximate_sync_.reset(
        new ApproximateSync(
          ApproximatePolicy(queue_size_),
          *left_sub_, *right_sub_, *left_info_sub_, *right_info_sub_));
      approximate_sync_->registerCallback(cb);
    } else {
      exact_sync_.reset(
        new ExactSync(
          ExactPolicy(queue_size_),
          *left_sub_, *right_sub_, *left_info_sub_, *right_info_sub_));
      exact_sync_->registerCallback(cb);
    }
  }

  template<class C>
  Connection addCallback(const C & callback)
  {
    return signal_.addCallback(callback);
  }
};

} // namespace viso2_ros
