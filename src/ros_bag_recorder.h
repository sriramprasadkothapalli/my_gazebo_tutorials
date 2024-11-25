// simple_bag_recorder.h
#pragma once

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <memory>

/**
 * @brief Class for recording messages to a rosbag.
 * 
 * The RosBagRecorder class subscribes to the /cmd_vel topic and records incoming messages to a rosbag.
 */
class RosBagRecorder : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Simple Bag Recorder object.
   * 
   * Initializes the subscription to the /cmd_vel topic and sets up the rosbag writer.
   */
  RosBagRecorder();

 private:
  /**
   * @brief Callback function for recording messages from the /cmd_vel topic.
   * 
   * @param msg Pointer to the serialized message to be recorded.
   */
  void topic_callback(std::shared_ptr<geometry_msgs::msg::Twist> msg) const;

  // Member Variables
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_; ///< Subscription to the /cmd_vel topic.
  std::unique_ptr<rosbag2_cpp::Writer> writer_;                             ///< Unique pointer to the rosbag writer.
};

