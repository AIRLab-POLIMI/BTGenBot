#ifndef BUILD_MOVE_TO_ASYNC_HPP
#define BUILD_MOVE_TO_ASYNC_HPP

#include "behaviortree_cpp/action_node.h"

#include <chrono>
#include <thread>
#include <utility>

namespace chr = std::chrono;

class MoveToAsync : public BT::StatefulActionNode
{
public:
    // Any TreeNode with ports must have a constructor with this signature
    MoveToAsync(const std::string& name, const BT::NodeConfig& config)
            : StatefulActionNode(name, config)
    {}

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("x"),  BT::InputPort<std::string>("y")};
    }

    // this function is invoked once at the beginning.
    BT::NodeStatus onStart() override;

    // If onStart() returned RUNNING, we will keep calling
    // this method until it return something different from RUNNING
    BT::NodeStatus onRunning() override;

    // callback to execute if the action was aborted by another node
    void onHalted() override;

private:
    chr::system_clock::time_point _completion_time;
};

//-------------------------

BT::NodeStatus MoveToAsync::onStart()
{
    BT::Expected<float> x = getInput<float>("x");
    BT::Expected<float> y = getInput<float>("y");

    std::cout<<"[ MoveBase: SEND REQUEST ]. goal: x="<<x.value()<<" y="<<y.value()<<std::endl;

    // We use this counter to simulate an action that takes a certain
    // amount of time to be completed (200 ms)
    _completion_time = chr::system_clock::now() + chr::milliseconds(220);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveToAsync::onRunning()
{
    // Pretend that we are checking if the reply has been received
    // you don't want to block inside this function too much time.
    std::this_thread::sleep_for(chr::milliseconds(10));

    // Pretend that, after a certain amount of time,
    // we have completed the operation
    if(chr::system_clock::now() >= _completion_time)
    {
        std::cout << "[ MoveBase: FINISHED ]" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void MoveToAsync::onHalted()
{
    std::cout << "[ MoveBase: ABORTED ]" << std::endl;
}

#endif //BUILD_MOVE_TO_ASYNC_HPP
