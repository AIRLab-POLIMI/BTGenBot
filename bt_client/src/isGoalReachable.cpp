/**
 * @file isGoalReachable.cpp
 * @author Riccardo Andrea Izzo (riccardo.izzo@mail.polimi.it)
 * @version 0.1
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "isGoalReachable.hpp"

isGoalReachable::isGoalReachable(const std::string& name, 
    const NodeConfig& config)
    : StatefulActionNode(name, config)
{
}

PortsList isGoalReachable::providedPorts()
{
    // prob input identify the probability of a goal pose to be reachable
    return{ BT::InputPort<float>("prob") };
}

NodeStatus isGoalReachable::onStart()
{   
    // Declare a variable to store the probability input
    float prob;
    getInput("prob", prob);
    
    // Mock to check if the goal is reachable or not given the initial probability
    isReachable = (rand() % 100) <= (int) prob * 100;

    // Set a deadline for the execution based on the current time and delay
    deadline = system_clock::now() + seconds(DELAY);
    return NodeStatus::RUNNING;
}

NodeStatus isGoalReachable::onRunning()
{   
    // Check if the current time has exceeded the deadline
    if (system_clock::now() >= deadline) {
        // Check if the goal is reachable
        if(isReachable){
            std::cout << "The goal is reachable!" << std::endl;
            return NodeStatus::SUCCESS;
        }
        else{
            std::cout << "The goal is not reachable! Skipping to the next action..." << std::endl;
            return NodeStatus::FAILURE;
        }
    }
    // If the deadline has not been exceeded, keep running
    else {
        return NodeStatus::RUNNING;
    }
}

void isGoalReachable::onHalted()
{
    // No specific actions defined for feedback during the execution halt
}