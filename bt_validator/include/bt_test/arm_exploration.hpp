#ifndef BUILD_ARM_EXPLORATION_HPP
#define BUILD_ARM_EXPLORATION_HPP

#include "behaviortree_cpp/action_node.h"

#include <chrono>
#include <thread>
#include <utility>

namespace T6
{
    class Report
    {
    private:
        int _counter;

    public:
        explicit Report(int counter)
        {
            _counter = counter;
        }

        BT::NodeStatus nextConfiguration()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::cout << "Configurations remaining " << _counter << std::endl;
            _counter--;
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus checkForTarget() const
        {
            if (_counter < 0)
            {
                std::cout << "Target found -> SUCCESS" << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            std::cout << "Target not found -> CONTINUE" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
    };

    class ApproachTarget : public BT::SyncActionNode
    {
    public:
        // If your Node has ports, you must use this constructor signature
        ApproachTarget(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<Report> report)
            : SyncActionNode(name, config)
        {
            _report = std::move(report);
        }

        // It is mandatory to define this STATIC method.
        static BT::PortsList providedPorts()
        {
            return {};
        }

        // Override the virtual function tick()
        BT::NodeStatus tick() override
        {
            std::cout<<"Target reached"<<std::endl;
            return BT::NodeStatus::SUCCESS;
        }

    private:
        std::shared_ptr<Report> _report;
    };
}

#endif // BUILD_ARM_EXPLORATION_HPP
