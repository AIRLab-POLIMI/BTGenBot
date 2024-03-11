/**
 * @file MoveToWithTimeout.hpp
 * @author Riccardo Andrea Izzo (riccardo.izzo@mail.polimi.it)
 * @brief Action node that sends a navigation goal to the Nav2 action server, it wait until the timeout is reached
 * @version 0.1
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "behaviortree_ros2/bt_action_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "yaml-cpp/yaml.h"
#include <chrono>
#include <functional>
#include <string>

using namespace BT;

#define TIMEOUT 20

/**
 * @brief Definition of MoveToWithTimeout class inheriting from RosActionNode, it manages navigation goals with timeout
 * 
 */
class MoveToWithTimeout: public RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  MoveToWithTimeout(const std::string& name,
                const NodeConfig& conf,
                const RosNodeParams& params);

    static PortsList providedPorts();
    bool setGoal(Goal& goal) override;
    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback);
    NodeStatus onResultReceived(const WrappedResult& wr) override;
    virtual NodeStatus onFailure(ActionNodeErrorCode error) override;
private:
    // Shared pointer to the ROS 2 node for communication
    std::shared_ptr<rclcpp::Node> client;
    std::chrono::steady_clock::time_point start_time;
    std::chrono::seconds timeout_duration;
};