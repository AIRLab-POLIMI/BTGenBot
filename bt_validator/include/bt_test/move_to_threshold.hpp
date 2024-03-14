#ifndef BUILD_MOVETO_THRESHOLD_HPP
#define BUILD_MOVETO_THRESHOLD_HPP

#include "behaviortree_cpp/action_node.h"

#include <chrono>
#include <thread>
#include <utility>

namespace T2 {
    class Report
    {
    private:
        float _threshold;
        std::vector<std::pair<float, float>> _locations;
        std::vector<float> _values;
        bool belowThreshold;
    public:
        Report(float threshold, std::vector<std::pair<float, float>> locations, std::vector<float> values) {
            _threshold = threshold;
            belowThreshold = false;
            _locations = std::move(locations);
            _values = std::move(values);
        }
        bool verifyLocation(float x, float y) {
            auto loc = std::make_pair(x, y);
            auto position = std::find(_locations.begin(), _locations.end(), loc);
            if(position == _locations.end())
                return false;
            long index = position - _locations.begin();
            _locations.erase(position);
            float value = _values[index];
            _values.erase(_values.begin()+index);

            if(value >= _threshold && !belowThreshold) {
                return true;
            }
            if(value >= _threshold && belowThreshold) {
                return false;
            }
            if(value < _threshold) {
                belowThreshold = true;
                return true;
            }
        }

    };

// SyncActionNode (synchronous action) with an input port.
    class MoveToThreshold : public BT::SyncActionNode
    {
    public:
        // If your Node has ports, you must use this constructor signature
        MoveToThreshold(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Report> report)
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

#endif //BUILD_MOVETO_THRESHOLD_HPP
