#ifndef BUILD_MOVETO_HPP
#define BUILD_MOVETO_HPP

#include "behaviortree_cpp/action_node.h"

#include <chrono>
#include <thread>
#include <utility>

namespace T1 {
    class Report
    {
    private:
        std::vector<std::pair<float, float>> _locations;
        int _index;
    public:
        explicit Report(std::vector<std::pair<float, float>> locations) {
            _locations = std::move(locations);
            _index = 0;
        }
        bool verifyLocation(float x, float y) {
            if(_locations[_index].first == x && _locations[_index].second == y) {
                _index += 1;
                return true;
            } else {
                return false;
            }
        }

    };

    // SyncActionNode (synchronous action) with an input port.
    class MoveTo : public BT::SyncActionNode
    {
    public:
        // If your Node has ports, you must use this constructor signature
        MoveTo(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Report> report)
                : SyncActionNode(name, config)
        {
            _report = std::move(report);
        }

        // It is mandatory to define this STATIC method.
        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<std::string>("x"),  BT::InputPort<std::string>("y")};
        }

        // Override the virtual function tick()
        BT::NodeStatus tick() override
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            BT::Expected<float> x = getInput<float>("x");
            BT::Expected<float> y = getInput<float>("y");

            std::cout<<"Port value: ("<<x.value()<<" ,"<<y.value()<<")"<<std::endl;

            if(_report->verifyLocation(x.value(), y.value())) {
                std::cout<<"Location matches path -> SUCCESS"<<std::endl;
                return BT::NodeStatus::SUCCESS;
            } else {
                std::cout<<"Location does not match path -> FAILURE"<<std::endl;
                return BT::NodeStatus::FAILURE;
            }
        }
    private:
        std::shared_ptr<Report> _report;
    };
}

#endif //BUILD_MOVETO_HPP
