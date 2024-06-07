#ifndef ARUCO_DEMO_HPP
#define ARUCO_DEMO_HPP

#include "behaviortree_cpp/action_node.h"

#include <chrono>
#include <thread>
#include <utility>

namespace T10 {
    class Report
    {
    private:
        std::vector<std::string> _labels;
        int _arm_index;
        int _aruco_index;
        int _location_index;
    public:
        explicit Report(std::vector<std::string> labels) {
            _labels = std::move(labels);
            _arm_index = 0;
            _aruco_index = 0;
            _location_index = 0;
        }

        bool verifyLabel(std::string label, int type) {
            switch(type){
                case 0: //MoveTo
                    if(_labels[_location_index] == label) {
                        _location_index += 1;
                        return true;
                    } else {
                        return false;
                    }
                case 1: //FollowAruco
                    if(_labels[_aruco_index] == label) {
                        _aruco_index += 1;
                        return true;
                    } else {
                        return false;
                    }
                case 2: //MoveManipulator
                    if(_labels[_arm_index] == label) {
                        _arm_index += 1;
                        return true;
                    } else {
                        return false;
                    }
                default:
                    return false;
            }
        }
    };

    class MoveTo : public BT::SyncActionNode
    {
        public:
            MoveTo(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Report> report)
                    : SyncActionNode(name, config)
            {
                _report = std::move(report);
            }

            static BT::PortsList providedPorts()
            {
                return { BT::InputPort<std::string>("location") };
            }

            BT::NodeStatus tick() override {
                BT::Expected<std::string> label = getInput<std::string>("location");

                std::cout<<"Port value: ("<<label.value()<<")"<<std::endl;

                if(_report->verifyLabel(label.value(), 0)) {
                    std::cout<<"Robot successfully moved to "<<label.value()<<" -> SUCCESS"<<std::endl;
                    return BT::NodeStatus::SUCCESS;
                } else {
                    std::cout<<"Robot cannot move to "<<label.value()<<" -> FAILURE"<<std::endl;
                    return BT::NodeStatus::FAILURE;
                }
            }

        private:
            std::shared_ptr<Report> _report;
    };

    class MoveManipulator : public BT::SyncActionNode
    {
        public:
            MoveManipulator(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Report> report)
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

                if(_report->verifyLabel(label.value(), 1)) {
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

    class FollowAruco : public BT::SyncActionNode
    {
        public:
            FollowAruco(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Report> report)
                    : SyncActionNode(name, config)
            {
                _report = std::move(report);
            }

            static BT::PortsList providedPorts()
            {
                return { BT::InputPort<std::string>("id") };
            }

            BT::NodeStatus tick() override {
                BT::Expected<std::string> label = getInput<std::string>("id");

                std::cout<<"Port value: ("<<label.value()<<")"<<std::endl;

                if(_report->verifyLabel(label.value(), 2)) {
                    std::cout<<"Aruco with id "<<label.value()<<" found -> SUCCESS"<<std::endl;
                    return BT::NodeStatus::SUCCESS;
                } else {
                    std::cout<<"Aruco with id "<<label.value()<<" NOT found -> FAILURE"<<std::endl;
                    return BT::NodeStatus::FAILURE;
                }
            }

        private:
            std::shared_ptr<Report> _report;
    };
}

#endif //ARUCO_DEMO_HPP