/**
 * @file walker.cpp
 * @author Sriramprasad (sriram21@umd.edu)
 * @brief Algorithm to run the robot
 * @version 0.1
 * @date 2024-11-24
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "walker.h"
#include <sensor_msgs/msg/image.hpp>

using std::placeholders::_1;

/**
 * @brief Construct a new Walker object.
 * 
 * Initializes the subscription to the /demo_cam/mycamera/depth_demo topic.
 */
Walker::Walker() : Node("walker_node"), currentState_(std::make_shared<StopState>()) {
    // Create publisher for /cmd_vel topic
    auto pubTopicName = "cmd_vel";
    velocityPublisher_ = this->create_publisher<TWIST>(pubTopicName, 10);

    // Create subscriber for /demo_cam/mycamera/depth_demo topic
    auto subTopicName = "demo_cam/mycamera/depth_demo";
    auto subCallback = std::bind(&Walker::subscribe_callback, this, _1);
    imageSubscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subTopicName, 10, subCallback);

    // Create a timer for processing at 10Hz
    auto timerCallback = std::bind(&Walker::processCallback, this);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), timerCallback);
}

/**
 * @brief Subscription callback for image data from the /demo_cam/mycamera/depth_demo topic.
 * 
 * @param msg Shared pointer to the image message.
 */
void Walker::subscribe_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    lastImage_ = *msg;
}

/**
 * @brief Set the current state of the Walker.
 * 
 * @param newState The new state to set for the Walker.
 */
void Walker::setState(std::shared_ptr<WalkerState> newState) {
    currentState_ = newState;
}

/**
 * @brief Callback function for processing sensor data and handling state transitions.
 */
void Walker::processCallback() {
    // Do nothing until the first data read
    if (lastImage_.header.stamp.sec == 0) {
        return;
    }

    // Delegate behavior to the current state
    currentState_->handle(*this);
}

/**
 * @brief Function to check for obstacles in the path of the Walker.
 * 
 * @return true if an obstacle is detected, false otherwise.
 */
bool Walker::hasObstacle() {
    unsigned char* dataPtr = lastImage_.data.data();
    float* depthData = reinterpret_cast<float*>(dataPtr);
    int index = ((lastImage_.height - 40) / 2 * lastImage_.width) + lastImage_.width / 2;

    RCLCPP_INFO(this->get_logger(), "Depth data at index: %.2f", depthData[index]);
    if (depthData[index] < 0.5) {
        RCLCPP_INFO(this->get_logger(), "Obstacle detected at distance: %.2f", depthData[index]);
        return true;
    }
    return false;
}

// ForwardState Implementation
void ForwardState::handle(Walker& walker) {
    auto message = TWIST();
    message.linear.x = 0.2;
    walker.velocityPublisher_->publish(message);
    RCLCPP_INFO(walker.get_logger(), "State = FORWARD");

    if (walker.hasObstacle()) {
        walker.setState(std::make_shared<StopState>());
    }
}

// StopState Implementation
void StopState::handle(Walker& walker) {
    auto message = TWIST();
    walker.velocityPublisher_->publish(message);
    RCLCPP_INFO(walker.get_logger(), "State = STOP");

    if (walker.hasObstacle()) {
        walker.setState(std::make_shared<TurnState>());
    } else {
        walker.setState(std::make_shared<ForwardState>());
    }
}

// TurnState Implementation
void TurnState::handle(Walker& walker) {
    auto message = TWIST();
    if (walker.turnCount_ % 4 == 0) {
        message.angular.z = -0.4;
    } else {
        message.angular.z = 0.4;
    }
    walker.velocityPublisher_->publish(message);
    walker.turnCount_++;

    RCLCPP_INFO(walker.get_logger(), "State = TURN");
    rclcpp::sleep_for(std::chrono::milliseconds(4000));

    if (!walker.hasObstacle()) {
        walker.setState(std::make_shared<ForwardState>());
    }
}

/**
 * @brief Main function entry point.
 * 
 * Initializes ROS2, spins the Walker node, and shuts down cleanly.
 * 
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return int Exit status.
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Walker>());
    rclcpp::shutdown();
    return 0;
}
