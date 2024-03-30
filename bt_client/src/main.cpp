/**
 * @file main.cpp
 * @author Riccardo Andrea Izzo (riccardo.izzo@mail.polimi.it)
 * @brief Main function for the behavior tree client node
 * @version 0.1
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include <filesystem>

#include "MoveTo.hpp"
#include "MoveToWithTimeout.hpp"
#include "isGoalReachable.hpp"
#include "GenerateNextDestination.hpp"
#include "isExplorationComplete.hpp"
#include "FollowAruco.hpp"
#include "MoveManipulator.hpp"

/**
 * @brief Main function for the behavior tree client node
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
    // Get the current directory
    std::filesystem::path curr_dir = std::filesystem::current_path();
    // Get the configuration file path
    std::filesystem::path config_dir = curr_dir / "config/tree.yaml";
    // Load the configuration file with the tree name
    YAML::Node config_file = YAML::LoadFile(config_dir);

    // Get the directory of the behavior tree XML file using the name from the configuration file
    std::filesystem::path tree_dir = curr_dir / "bt_xml/";
    std::string tree_xml = tree_dir.string() + config_file["tree_name"].as<std::string>();

    rclcpp::init(argc, argv);

    BehaviorTreeFactory factory;

    auto main_node = std::make_shared<rclcpp::Node>("bt_client_node");
	// Declare a parameter for the location file with a default value of "none"
    main_node->declare_parameter("location_file","none");

    RosNodeParams params; 
    params.nh = main_node;
    params.default_port_value = "/navigate_to_pose";

    // Register the custom MoveTo node type with the BehaviorTreeFactory
    factory.registerNodeType<MoveTo>("MoveTo", params);

    // Register the custom MoveToWithTimeout node type with the BehaviorTreeFactory
    factory.registerNodeType<MoveToWithTimeout>("MoveToWithTimeout", params);

    // Register the custom isGoalReachable node type with the BehaviorTreeFactory
    factory.registerNodeType<isGoalReachable>("isGoalReachable");

    // Register the custom GenerateNextDestination node type with the BehaviorTreeFactory
    factory.registerNodeType<GenerateNextDestination>("GenerateNextDestination");

    // Register the custom isExplorationComplete node type with the BehaviorTreeFactory
    factory.registerNodeType<isExplorationComplete>("isExplorationComplete");

    auto aruco_node = std::make_shared<rclcpp::Node>("bt_aruco_client_node");
    RosNodeParams aruco_params; 
    aruco_params.nh = aruco_node;
    aruco_params.default_port_value = "/aruco";

    // Register the custom FollowAruco node type with the BehaviorTreeFactory
    factory.registerNodeType<FollowAruco>("FollowAruco", aruco_params);

    auto manipulator_node = std::make_shared<rclcpp::Node>("bt_manipulator_client_node");
    RosNodeParams manipulator_params; 
    manipulator_params.nh = manipulator_node;
    manipulator_params.default_port_value = "/manipulator";

    // Register the custom ResetManipulator node type with the BehaviorTreeFactory
    factory.registerNodeType<MoveManipulator>("MoveManipulator", manipulator_params);

    // Check if the file exists
    if (!std::filesystem::exists(tree_xml)) {
        RCLCPP_ERROR(main_node->get_logger(), "Tree file %s does not exist!", tree_xml.c_str());
        return 1;
    }

    // Create a behavior tree from an XML file located in the specified directory
    auto tree = factory.createTreeFromFile(tree_xml);

    // Execute the behavior tree and tick while it is running
    tree.tickWhileRunning();

    return 0;
}