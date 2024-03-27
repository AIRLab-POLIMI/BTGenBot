/**
 * @file bt_monitor.cpp
 * @author Riccardo Andrea Izzo (riccardo.izzo@mail.polimi.it)
 * @brief Monitor functions that waits for the behavior tree XML file to be generated by the LLM, 
 * when it is ready it launches the behavior tree client node
 * @version 0.1
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <chrono>
#include <memory>
#include <iostream>
#include <cstdlib> 
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_interfaces/node_topics.hpp"
#include "rcpputils/filesystem_helper.hpp"

using namespace std::chrono_literals;

const std::string dir = "/BTGenBot/bt_client/bt_xml/";

const std::string tree_xml = "main_tree.xml";



class FileCheckerNode : public rclcpp::Node
{
public:
    FileCheckerNode() : Node("file_checker")
    {
        timer_ = this->create_wall_timer(1000ms, std::bind(&FileCheckerNode::checkFile, this));
    }

private:
    void checkFile()
    {
        if (rcpputils::fs::exists(dir + tree_xml))
        {
            RCLCPP_INFO(this->get_logger(), "File found. Launching behaviour tree client...");
            std::system("ros2 launch bt_client bt.launch.py");
            timer_->cancel(); 
            rclcpp::shutdown(); 
        }
    }

    std::string file_path_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FileCheckerNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
