// walker.h
#ifndef WALKER_H
#define WALKER_H

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>

using IMAGE = sensor_msgs::msg::Image;
using TWIST = geometry_msgs::msg::Twist;

class Walker;

/**
 * @brief State Interface for Walker states.
 * 
 * This interface defines the common behavior for all states of the Walker.
 */
class WalkerState {
public:
    virtual ~WalkerState() = default;

    /**
     * @brief Handle the behavior for the current state.
     * 
     * @param walker Reference to the Walker instance to interact with.
     */
    virtual void handle(Walker& walker) = 0;
};

/**
 * @brief Walker Class Definition
 * 
 * The Walker class represents the robot, managing its state and behavior.
 * It communicates with ROS2 nodes to publish velocity commands and process sensor data.
 */
class Walker : public rclcpp::Node {
public:
    /**
     * @brief Construct a new Walker object.
     * 
     * Initializes the publisher, subscriber, and timer to control the robot's movement.
     */
    Walker();

    /**
     * @brief Set the current state of the Walker.
     * 
     * @param newState The new state to set for the Walker.
     */
    void setState(std::shared_ptr<WalkerState> newState);

    /**
     * @brief Callback function for processing sensor data and handling state transitions.
     */
    void processCallback();

    /**
     * @brief Subscription callback for image data from the /demo_cam/mycamera/depth_demo topic.
     * 
     * @param msg Shared pointer to the image message.
     */
    void subscribe_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    /**
     * @brief Function to check for obstacles in the path of the Walker.
     * 
     * @return true if an obstacle is detected, false otherwise.
     */
    bool hasObstacle();

    // Member Variables
    rclcpp::Subscription<IMAGE>::SharedPtr imageSubscription_; ///< Subscription to the camera depth topic.
    rclcpp::Publisher<TWIST>::SharedPtr velocityPublisher_;    ///< Publisher for the velocity command topic.
    rclcpp::TimerBase::SharedPtr timer_;                       ///< Timer for periodic state processing.
    IMAGE lastImage_;                                         ///< Last received image data.
    std::shared_ptr<WalkerState> currentState_;               ///< Current state of the Walker.
    int turnCount_ = 0;                                       ///< Counter to track the number of turns made.
};

/**
 * @brief State representing the Forward movement of the Walker.
 */
class ForwardState : public WalkerState {
public:
    /**
     * @brief Handle the behavior for the Forward state.
     * 
     * @param walker Reference to the Walker instance to interact with.
     */
    void handle(Walker& walker) override;
};

/**
 * @brief State representing the Stop behavior of the Walker.
 */
class StopState : public WalkerState {
public:
    /**
     * @brief Handle the behavior for the Stop state.
     * 
     * @param walker Reference to the Walker instance to interact with.
     */
    void handle(Walker& walker) override;
};

/**
 * @brief State representing the Turning behavior of the Walker.
 */
class TurnState : public WalkerState {
public:
    /**
     * @brief Handle the behavior for the Turn state.
     * 
     * @param walker Reference to the Walker instance to interact with.
     */
    void handle(Walker& walker) override;
};

#endif // WALKER_H
