/**
 * @file GenerateNextDestination.hpp
 * @author Riccardo Andrea Izzo (riccardo.izzo@mail.polimi.it)
 * @brief Action node that generates a new destination in order to explore the environment
 * @version 0.1
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "behaviortree_ros2/bt_action_node.hpp"
#include <chrono>
#include <cstdlib> 

using namespace BT;
using namespace std::chrono;

/**
 * @brief Definition of GenerateNextDestination class inheriting from SyncActionNode, it generates new destinations
 * 
 */
class GenerateNextDestination : public SyncActionNode {
public:
    /**
     * @brief Construct a new GenerateNextDestination object
     * 
     * @param name Name of the action node
     * @param config Node configuration
     */
    GenerateNextDestination(const std::string& name, const NodeConfig& config);

    /**
     * @brief Define the provided input ports for the GenerateNextDestination action
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