#ifndef BUILD_ASSEMBLY_HPP
#define BUILD_ASSEMBLY_HPP

#include "behaviortree_cpp/action_node.h"

#include <chrono>
#include <thread>
#include <utility>

namespace T9 {
    enum item {
        A,
        B,
        C,
        none
    };

    enum location {
        ST_A,
        ST_B,
        ST_C,
        ST_D,
        PARK
    };

    enum target {
        tray,
        station
    };

    class Robot
    {
    private:
        location _currentLocation = location::PARK;
        item _hand = item::none;
        std::vector<item> _tray;
    public:
        Robot() {
            _tray = {};
        }

        location currentLocation() {
            return _currentLocation;
        }

        void moveTo(location location) {
            _currentLocation = location;
        }

        bool pickFromTray(item item) {
            auto it = std::find(_tray.begin(), _tray.end(), item);
            if(it != _tray.end()) {
                _hand = item;
                _tray.erase(it); // remove the item from the tray
                return true;
            } else {
                return false;
            }
        }

        bool pickFromStation(item object, std::vector<item> &station) {
            auto it = std::find(station.begin(), station.end(), object);
            if(it != station.end()) {
                _hand = object;
                station.erase(it); // remove the item from the station
                return true;
            } else {
                return false;
            }
        }

        bool dropIntoTray() {
            if(_hand != item::none) {
                _tray.push_back(_hand);
                _hand = item::none;
                return true;
            } else {
                return false;
            }
        }

        bool dropToStation(std::vector<item> &station) {
            if(_hand != item::none) {
                station.push_back(_hand);
                _hand = item::none;
                return true;
            } else {
                return false;
            }
        }

        bool DropToTray(item item) {
            if(_hand != item::none) {
                _tray.push_back(_hand);
                _hand = item::none;
                return true;
            } else {
                return false;
            }
        }
    };

    class Report
    {
    private:
        std::vector<item> _stA, _stB, _stC;
        Robot _robot;

        location locationFromString(std::string location) {
            if(location == "Station A") {
                return location::ST_A;
            } else if(location == "Station B") {
                return location::ST_B;
            } else if(location == "Station C") {
                return location::ST_C;
            } else if(location == "Station D") {
                return location::ST_D;
            } else if(location == "Park") {
                return location::PARK;
            } else {
                return location::PARK;
            }
        }

        item itemFromString(std::string item) {
            if(item == "Component A") {
                return item::A;
            } else if(item == "Component B") {
                return item::B;
            } else if(item == "Finished product") {
                return item::C;
            } else {
                return item::none;
            }
        }

        target targetFromString(std::string target) {
            if(target == "tray") {
                return target::tray;
            } else if(target == "station") {
                return target::station;
            } else {
                return target::tray;
            }
        }

    public:
        explicit Report() {
            _stA = {item::A};
            _stB = {item::B};
            _stC = {};
        }

        BT::NodeStatus MoveTo(std::string location) {
            std::cout<<"Robot moved to "<<location<<std::endl;
            _robot.moveTo(locationFromString(location));
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus Pick(std::string target, std::string item) {
            switch (targetFromString(target))
            {
            case target::tray:
                if(_robot.pickFromTray(itemFromString(item))) {
                    std::cout<<"Robot picked "<<item<<" from tray"<<std::endl;
                    return BT::NodeStatus::SUCCESS;
                } else {
                    std::cout<<"Robot could not pick "<<item<<" from tray"<<std::endl;
                    return BT::NodeStatus::FAILURE;
                }
                break;
            case target::station:
            {
                bool result;
                switch (_robot.currentLocation())
                {
                case location::ST_A:
                    result = _robot.pickFromStation(itemFromString(item), _stA);
                    break;
                case location::ST_B:
                    result = _robot.pickFromStation(itemFromString(item), _stB);
                    break;
                case location::ST_C:
                    result = _robot.pickFromStation(itemFromString(item), _stC);
                    break;
                default:
                    break;
                }
                if(result) {
                    std::cout<<"Robot picked "<<item<<" from station"<<std::endl;
                    return BT::NodeStatus::SUCCESS;
                } else {
                    std::cout<<"Robot could not pick "<<item<<" from station"<<std::endl;
                    return BT::NodeStatus::FAILURE;
                }
                break;
            }
                break;
            default:
                break;
            }
        }

        BT::NodeStatus Drop(std::string target) {
            switch(targetFromString(target)) {
                case target::tray:
                    if(_robot.dropIntoTray()) {
                        std::cout<<"Robot dropped item into tray"<<std::endl;
                        return BT::NodeStatus::SUCCESS;
                    } else {
                        std::cout<<"Robot could not drop item into tray"<<std::endl;
                        return BT::NodeStatus::FAILURE;
                    }
                    break;
                case target::station:
                {
                    bool result;
                    switch (_robot.currentLocation())
                    {
                    case location::ST_A:
                        result = _robot.dropToStation(_stA);
                        break;
                    case location::ST_B:
                        result = _robot.dropToStation(_stB);
                        break;
                    case location::ST_C:
                        result = _robot.dropToStation(_stC);
                        break;
                    case location::PARK:
                        result = true;
                        break;
                    default:
                        break;
                    }
                    if(result) {
                        std::cout<<"Robot dropped item into station"<<std::endl;
                        return BT::NodeStatus::SUCCESS;
                    } else {
                        std::cout<<"Robot could not drop item into station"<<std::endl;
                        return BT::NodeStatus::FAILURE;
                    }
                    break;
                }
            }

        }

        BT::NodeStatus ActivateProcess() {
            std::cout<<"Process activated"<<std::endl;
            auto itA = std::find(_stC.begin(), _stC.end(), item::A);
            auto itB = std::find(_stC.begin(), _stC.end(), item::B);
            if(itA != _stC.end() && itB != _stC.end()) {
                std::cout<<"Process completed"<<std::endl;
                _stC.clear();
                _stC.push_back(item::C);
                return BT::NodeStatus::SUCCESS;
            } else {
                std::cout<<"Process not completed"<<std::endl;
                return BT::NodeStatus::FAILURE;
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
                return _report->MoveTo(getInput<std::string>("location").value());
            }

        private:
            std::shared_ptr<Report> _report;
    };

    class Pick : public BT::SyncActionNode {
        public:
            Pick(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Report> report)
                    : SyncActionNode(name, config)
            {
                _report = std::move(report);
            }

            static BT::PortsList providedPorts()
            {
                return { BT::InputPort<std::string>("from"), BT::InputPort<std::string>("item") };
            }

            BT::NodeStatus tick() override
            {
                return _report->Pick(getInput<std::string>("from").value(), getInput<std::string>("item").value());
            }
        private:
            std::shared_ptr<Report> _report;
    };

    class Drop : public BT::SyncActionNode {
        public:
            Drop(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Report> report)
                    : SyncActionNode(name, config)
            {
                _report = std::move(report);
            }

            static BT::PortsList providedPorts()
            {
                return { BT::InputPort<std::string>("to") };
            }

            BT::NodeStatus tick() override {
                return _report->Drop(getInput<std::string>("to").value());
            }
        private:
            std::shared_ptr<Report> _report;
    };
}

#endif //BUILD_ASSEMBLY_HPP