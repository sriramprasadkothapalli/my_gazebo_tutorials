/**
 * @file ros_bag_recorder.cpp
 * @author Sriramprasad (sriram21@umd.edu)
 * @brief Algorithm to run the robot
 * @version 0.1
 * @date 2024-11-24
 * 
 * @copyright Copyright (c) 2024
 * 
 */

// ros_bag_recorder.cpp
#include "ros_bag_recorder.h"

using std::placeholders::_1;

/**
 * @brief Construct a new Simple Bag Recorder object.
 * 
 * Initializes the subscription to the /cmd_vel topic and sets up the rosbag writer.
 */
RosBagRecorder::RosBagRecorder() : Node("simple_bag_recorder") {
    // Set up rosbag writer
    writer_ = std::make_unique<rosbag2_cpp::Writer>();

    // Open a rosbag named "my_bag"
    writer_->open("my_bag");

    // Create subscription to the /cmd_vel topic
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&RosBagRecorder::topic_callback, this, _1));
}

/**
 * @brief Callback function for recording messages from the /cmd_vel topic.
 * 
 * @param msg Pointer to the serialized message to be recorded.
 */
void RosBagRecorder::topic_callback(std::shared_ptr<geometry_msgs::msg::Twist> msg) const {
    rclcpp::Serialization<geometry_msgs::msg::Twist> serializer;
    auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();

    // Serialize the message
    serializer.serialize_message(msg.get(), serialized_msg.get());

    // Get the current time for the message timestamp
    rclcpp::Time time_stamp = this->now();

    // Write the serialized message to the bag file using the recommended method
    writer_->write(serialized_msg, "cmd_vel", "geometry_msgs/msg/Twist", time_stamp);
}
/**
 * @brief Main function entry point.
 * 
 * Initializes ROS2, spins the RosBagRecorder node, and shuts down cleanly.
 * 
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return int Exit status.
 */
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RosBagRecorder>());
    rclcpp::shutdown();
    return 0;
}
