/**
 * @file isExplorationComplete.hpp
 * @author Riccardo Andrea Izzo (riccardo.izzo@mail.polimi.it)
 * @brief Action node that checks if the exploration is complete
 * @version 0.1
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "behaviortree_ros2/bt_action_node.hpp"
#include <chrono>
#include <cstdlib> 

#define NUM_LOCATIONS 2

using namespace BT;
using namespace std::chrono;

/**
 * @brief Definition of isExplorationComplete class inheriting from SyncActionNode, it checks if the exploration is complete
 * 
 */
class isExplorationComplete : public SyncActionNode {
public:
    /**
     * @brief Construct a new isExplorationComplete object
     * 
     * @param name Name of the action node
     * @param config Node configuration
     */
    isExplorationComplete(const std::string& name, const NodeConfig& config);

    /**
     * @brief Define the provided input ports for the isExplorationComplete action
     * 
     * @return PortsList List of input ports 
     */
    static PortsList providedPorts();

    /**
     * @brief Handle the node's behavior
     * 
     * @return NodeStatus Status of the node
     */
    NodeStatus tick() override;
private:
};