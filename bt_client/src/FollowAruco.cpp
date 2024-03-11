/**
 * @file FollowAruco.cpp
 * @author Riccardo Andrea Izzo (riccardo.izzo@mail.polimi.it)
 * @version 0.1
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "FollowAruco.hpp"

FollowAruco::FollowAruco(const std::string& name,
    const NodeConfig& conf,
    const RosNodeParams& params)
    : RosActionNode<Aruco>(name, conf, params)
{
    client = params.nh;
}

PortsList FollowAruco::providedPorts()
{
    return{ BT::InputPort<int>("id")};
}

bool FollowAruco::setGoal(RosActionNode::Goal &goal)
{
    float id;
    getInput("id", id);
    // Set the goal for the action
    goal.aruco_id = id;

    return true;
}

NodeStatus FollowAruco::onResultReceived(const RosActionNode::WrappedResult &wr)
{   
    RCLCPP_INFO(client->get_logger(), "Goal reached\n");
    return NodeStatus::SUCCESS;
}

NodeStatus FollowAruco::onFailure(ActionNodeErrorCode error)
{   
    RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
    return NodeStatus::FAILURE;
}

NodeStatus FollowAruco::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
    return NodeStatus::RUNNING;
}