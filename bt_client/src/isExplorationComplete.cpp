/**
 * @file isExplorationComplete.cpp
 * @author Riccardo Andrea Izzo (riccardo.izzo@mail.polimi.it)
 * @version 0.1
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "isExplorationComplete.hpp"

isExplorationComplete::isExplorationComplete(const std::string& name, 
    const NodeConfig& config)
    : SyncActionNode(name, config)
{
}

PortsList isExplorationComplete::providedPorts()
{
    // location input that identified the goal location
    return{ BT::InputPort<std::string>("location")};
}


NodeStatus isExplorationComplete::tick()
{   
    // Store the goal location
    std::string goalLocation;
    getInput("location", goalLocation);

    // Check if the goal location is the parking, if not the exploration is not complete
    if(goalLocation != "Parking"){
        std::cout<<"Exploration not complete. Going on ..."<<std::endl;
        return NodeStatus::FAILURE;
    }
    // If the goal location is the parking, the exploration is complete
    else {
        std::cout<<"Exploration complete! "<<std::endl;
        return NodeStatus::SUCCESS;
    }
}