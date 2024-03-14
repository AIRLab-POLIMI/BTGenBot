/**
 * @file MoveManipulator.cpp
 * @author Riccardo Andrea Izzo (riccardo.izzo@mail.polimi.it)
 * @version 0.1
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "MoveManipulator.hpp"

MoveManipulator::MoveManipulator(const std::string& name,
    const NodeConfig& conf,
    const RosNodeParams& params)
    : RosActionNode<Manipulator>(name, conf, params)
{
    client = params.nh;
}

PortsList MoveManipulator::providedPorts()
{
    return{ BT::InputPort<std::string>("state")};
}

bool MoveManipulator::setGoal(RosActionNode::Goal &goal)
{
    std::string goal_state;
    getInput("state", goal_state);
    
    // Set the goal for the action
    goal.state = goal_state;

    return true;
}

NodeStatus MoveManipulator::onResultReceived(const RosActionNode::WrappedResult &wr)
{   
    RCLCPP_INFO(client->get_logger(), "Goal reached\n");
    return NodeStatus::SUCCESS;
}

NodeStatus MoveManipulator::onFailure(ActionNodeErrorCode error)
{   
    RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
    return NodeStatus::FAILURE;
}

NodeStatus MoveManipulator::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
    return NodeStatus::RUNNING;
}