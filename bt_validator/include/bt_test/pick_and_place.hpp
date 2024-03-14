#ifndef BUILD_PICK_AND_PLACE_HPP
#define BUILD_PICK_AND_PLACE_HPP

#include "behaviortree_cpp/action_node.h"

#include <chrono>
#include <thread>
#include <utility>

namespace T7 {
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
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            _index++;
            std::cout<<"Current index "<<_index<<std::endl;
            if(_index > (int) _locations.size() - 1) {
                std::cout<<"Location generated when exploration complete -> FAILURE"<<std::endl;
                return BT::NodeStatus::FAILURE;
            }
            std::cout<<"Location generated correctly -> SUCCESS"<<std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus explorationComplete() {
            if(_index >= (int) _locations.size() - 1) {
                std::cout<<"Exploration complete -> SUCCESS"<<std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            std::cout<<"Exploration not complete -> CONTINUE"<<std::endl;
            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus moveTo() {
            std::cout<<"Moving to destination "<<_locations[_index].first<<", "<<_locations[_index].second<<std::endl;
            return BT::NodeStatus::SUCCESS;
        }

    };
}


#endif //BUILD_PICK_AND_PLACE_HPP
