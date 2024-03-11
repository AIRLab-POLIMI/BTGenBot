/**
 * @file isGoalReachable.hpp
 * @author Riccardo Andrea Izzo (riccardo.izzo@mail.polimi.it)
 * @brief Action node that checks the reachibility of a given goal
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

#define DELAY 10

/**
 * @brief Definition of isGoalReachable class inheriting from StatefulActionNode, it checks the reachibility of a given goal
 * 
 */
class isGoalReachable : public StatefulActionNode {
public:
    /**
     * @brief Construct a new isGoalReachable object
     * 
     * @param name Name of the action node
     * @param config Node configuration
     */
    isGoalReachable(const std::string& name, const NodeConfig& config);

    /**
     * @brief Define the provided input ports for the isGoalReachable action
     * 
     * @return PortsList List of input ports 
     */
    static PortsList providedPorts();

    /**
     * @brief Handle the node's behavior when it starts its execution
     * 
     * @return NodeStatus Status of the node
     */
    NodeStatus onStart() override;

    /**
     * @brief Handle the node's behavior while it is running
     * 
     * @return NodeStatus Status of the node
     */
    NodeStatus onRunning() override;

    /**
     * @brief Handle any feedback during the execution of the node
     * 
     */
    virtual void onHalted() override;
private:
    // Store the deadline for the execution
    system_clock::time_point deadline;
    // Store the reachibility of the goal
    bool isReachable;
};