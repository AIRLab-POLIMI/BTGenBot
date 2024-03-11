/**
 * @file MoveTo.hpp
 * @author Riccardo Andrea Izzo (riccardo.izzo@mail.polimi.it)
 * @brief Action node that sends a navigation goal to the Nav2 action server
 * @version 0.1
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <optional>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "yaml-cpp/yaml.h"

using N2P = nav2_msgs::action::NavigateToPose;
using GoalHandleN2P = rclcpp_action::ClientGoalHandle<N2P>;

using namespace BT;

/**
 * @brief Definition of MoveTo class inheriting from RosActionNode, it manages navigation goals
 * 
 */
class MoveTo: public RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
    /**
     * @brief Construct a new MoveTo object
     * 
     * @param name Name of the action node
     * @param conf Node configuration
     * @param params ROS node parameters
     */
    MoveTo(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);
    
    /**
     * @brief Define the provided input ports for the MoveTo action
     * 
     * @return PortsList List of input ports 
     */
    static PortsList providedPorts();

    /**
     * @brief Set the goal for the MoveTo action based on the specified location
     * 
     * @param goal navioal to be set
     * @return true if the goal is set successfully
     * @return false if the goal is not set successfully
     */
    bool setGoal(Goal& goal) override;

    /**
     * @brief Handle the feedback received during the execution of the MoveTo action
     * 
     * @param feedback Feedback received
     * @return NodeStatus
     */
    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback);

    /**
     * @brief Handle the result received after the execution of the MoveTo action
     * 
     * @param wr Wrapped result received
     * @return NodeStatus 
     */
    NodeStatus onResultReceived(const WrappedResult& wr) override;

    /**
     * @brief Handle the failure of the MoveTo action
     * 
     * @param error Error code
     * @return NodeStatus 
     */
    virtual NodeStatus onFailure(ActionNodeErrorCode error) override;
private:
    // Shared pointer to the ROS 2 node for communication
    std::shared_ptr<rclcpp::Node> client;
};
