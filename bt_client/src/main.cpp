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
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "MoveTo.hpp"
#include "MoveToWithTimeout.hpp"
#include "isGoalReachable.hpp"
#include "GenerateNextDestination.hpp"
#include "isExplorationComplete.hpp"
#include "FollowAruco.hpp"
#include "MoveManipulator.hpp"

// Define the directory for behavior tree XML files
const std::string bt_xml_dir = ament_index_cpp::get_package_share_directory("bt_client") + "/bt_xml";

// Select here the behavior tree
const std::string tree_xml = "/demo_task.xml";

/**
 * @brief Main function for the behavior tree client node
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
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

    // Create a behavior tree from an XML file located in the specified directory
    auto tree = factory.createTreeFromFile(bt_xml_dir + tree_xml);

    // Execute the behavior tree and tick while it is running
    tree.tickWhileRunning();

    return 0;
}