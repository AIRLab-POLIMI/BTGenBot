/**
 * @file MoveToWithTimeout.cpp
 * @author Riccardo Andrea Izzo (riccardo.izzo@mail.polimi.it)
 * @version 0.1
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "MoveToWithTimeout.hpp"

MoveToWithTimeout::MoveToWithTimeout(const std::string& name,
    const NodeConfig& conf,
    const RosNodeParams& params)
    : RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
{
    client = params.nh;
    // Set the timeout duration
    timeout_duration = std::chrono::seconds(TIMEOUT);
}

// Define the provided input ports for the MoveToWithTimeout action
PortsList MoveToWithTimeout::providedPorts()
{
    return {
        BT::InputPort<std::string>("location"),
    };
}

// Set the goal for the MoveToWithTimeout action based on the specified location
bool MoveToWithTimeout::setGoal(RosActionNode::Goal &goal)
{
    // Retrieve the location from the input port
    std::string loc;
    getInput("location", loc);

    // Load the YAML file containing location information
    const std::string file_path = client->get_parameter("location_file").as_string();
    YAML::Node locations = YAML::LoadFile(file_path);

    // Check if the location exists in the YAML file
    if (!locations[loc])
    {
      RCLCPP_ERROR(client->get_logger(), "Location '%s' not found in the YAML file", loc.c_str());
      return false;
    }

    // Extract the coordinates for the specified location
    std::vector<float> current_goal = locations[loc].as<std::vector<float>>();

    goal.pose.header.stamp = client->now();
    goal.pose.header.frame_id = "map";

    goal.pose.pose.position.x = current_goal[0];
    goal.pose.pose.position.y = current_goal[1];
    goal.pose.pose.orientation.x = 0.0;
    goal.pose.pose.orientation.y = 0.0;
    goal.pose.pose.orientation.w = 1.0;
    goal.pose.pose.orientation.z = 0.0;

    start_time = std::chrono::steady_clock::now();

    return true;
}

// Handle the result received after executing the MoveTo action
NodeStatus MoveToWithTimeout::onResultReceived(const RosActionNode::WrappedResult &wr)
{   
    RCLCPP_INFO(client->get_logger(), "Goal reached\n");
    return NodeStatus::SUCCESS;
}

// Handle failure cases for the MoveTo action
NodeStatus MoveToWithTimeout::onFailure(ActionNodeErrorCode error)
{   
    RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
    return NodeStatus::FAILURE;
}

// Handle feedback during the execution of the MoveTo action
NodeStatus MoveToWithTimeout::onFeedback(const std::shared_ptr<const Feedback> feedback)
{   
    auto end_time = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);

    // Check if the action succeeded within the timeout
    if (elapsed_time >= timeout_duration)
    {   
        RCLCPP_ERROR(client->get_logger(), "Timeout exceeded (20s), goal not reached\n");
        return NodeStatus::FAILURE;
    }
    else
    {
        return NodeStatus::RUNNING;
    }
}
