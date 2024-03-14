#ifndef BUILD_EXPLORATION_HPP
#define BUILD_EXPLORATION_HPP

#include "behaviortree_cpp/action_node.h"

#include <chrono>
#include <thread>
#include <utility>

namespace T5 {
    class Report
    {
    private:
        std::vector<std::pair<float, float>> _locations;
        int _index;
    public:
        explicit Report(std::vector<std::pair<float, float>> locations) {
            _locations = std::move(locations);
            _index = -1;
        }

        BT::NodeStatus nextDestination() {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            _index++;
            if(_index > _locations.size() - 1) {
                std::cout<<"Location generated when exploration complete -> FAILURE"<<std::endl;
                return BT::NodeStatus::FAILURE;
            }
            std::cout<<"Location generated correctly -> SUCCESS"<<std::endl;

            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus explorationComplete() {
            if(_index >= _locations.size() - 1) {
                std::cout<<"Exploration complete -> SUCCESS"<<std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            std::cout<<"Exploration not complete -> CONTINUE"<<std::endl;

            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus moveTo() {
            std::cout<<"Current index "<<_index<<std::endl;
            std::cout<<"Moving to destination "<<_locations[_index].first<<", "<<_locations[_index].second<<std::endl;

            return BT::NodeStatus::SUCCESS;
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
            return {};
        }

        // Override the virtual function tick()
        BT::NodeStatus tick() override
        {
            return _report->moveTo();
        }
    private:
        std::shared_ptr<Report> _report;
    };
}

#endif //BUILD_EXPLORATION_HPP
