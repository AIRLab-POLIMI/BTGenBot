#ifndef BUILD_PRESS_BUTTON_HPP
#define BUILD_PRESS_BUTTON_HPP

#include "behaviortree_cpp/action_node.h"

#include <chrono>
#include <thread>
#include <utility>

namespace T8 {
    class Report
    {
    private:
        std::vector<std::string> _labels;
        int _index;
    public:
        explicit Report(std::vector<std::string> labels) {
            _labels = std::move(labels);
            _index = 0;
        }

        bool verifyLabel(std::string label) {
            if(_labels[_index] == label) {
                _index += 1;
                return true;
            } else {
                return false;
            }
        }
    };

    class PressButton : public BT::SyncActionNode
    {
        public:
            PressButton(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Report> report)
                    : SyncActionNode(name, config)
            {
                _report = std::move(report);
            }

            static BT::PortsList providedPorts()
            {
                return { BT::InputPort<std::string>("label") };
            }

            BT::NodeStatus tick() override {
                BT::Expected<std::string> label = getInput<std::string>("label");

                std::cout<<"Port value: ("<<label.value()<<")"<<std::endl;

                if(_report->verifyLabel(label.value())) {
                    std::cout<<"Label matches -> SUCCESS"<<std::endl;
                    return BT::NodeStatus::SUCCESS;
                } else {
                    std::cout<<"Label does not match -> FAILURE"<<std::endl;
                    return BT::NodeStatus::FAILURE;
                }
            }

        private:
            std::shared_ptr<Report> _report;
    };
}

#endif //BUILD_PRESS_BUTTON_HPP