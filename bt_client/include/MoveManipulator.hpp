/**
 * @file ResetManipulator.hpp
 * @author Riccardo Andrea Izzo (riccardo.izzo@mail.polimi.it)
 * @brief Action node that sends a goal to the manipulator_action_server
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
#include "igus_rebel_commander/action/move_manipulator.hpp"

using Manipulator = igus_rebel_commander::action::MoveManipulator;
using GoalHandleManipulator = rclcpp_action::ServerGoalHandle<Manipulator>;

using namespace BT;

/**
 * @brief Definition of MoveManipulator class inheriting from RosActionNode, it manages manipulator goal
 * 
 */
class MoveManipulator: public RosActionNode<Manipulator>
{
public:
    /**
     * @brief Construct a new MoveManipulator object
     * 
     * @param name Name of the action node
     * @param conf Node configuration
     * @param params ROS node parameters
     */
    MoveManipulator(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);

    /**
     * @brief Define the provided input ports for the MoveManipulator action
     * 
     * @return PortsList List of input ports 
     */
    static PortsList providedPorts();

    /**
     * @brief Set the goal for the MoveManipulator action based on the joint configuration
     * 
     * @param goal target joint configuration to be set
     * @return true if the goal is set successfully
     * @return false if the goal is not set successfully
     */
    bool setGoal(Goal& goal) override;

    /**
     * @brief Handle the feedback received during the execution of the MoveManipulator action
     * 
     * @param feedback Feedback received
     * @return NodeStatus Status of the action
     */
    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback);

    /**
     * @brief Handle the result received after executing the MoveManipulator action
     * 
     * @param wr Wrapped result received
     * @return NodeStatus Status of the action
     */
    NodeStatus onResultReceived(const WrappedResult& wr) override;

    /**
     * @brief Handle failure cases for the MoveManipulator action
     * 
     * @param error Error code received
     * @return NodeStatus Status of the action
     */
    virtual NodeStatus onFailure(ActionNodeErrorCode error) override;
private:
    // Shared pointer to the ROS 2 node for communication
    std::shared_ptr<rclcpp::Node> client;
};
