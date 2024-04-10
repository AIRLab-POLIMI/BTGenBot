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
#include <filesystem>
#include "yaml-cpp/yaml.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_interfaces/node_topics.hpp"
#include "rcpputils/filesystem_helper.hpp"



using namespace std::chrono_literals;


class FileCheckerNode : public rclcpp::Node
{
public:
    FileCheckerNode() : Node("file_checker")
    {
        // Create a timer to check the file every second
        timer_ = this->create_wall_timer(1000ms, std::bind(&FileCheckerNode::checkFile, this));

        // Get the current directory
        std::filesystem::path curr_dir = std::filesystem::current_path();
        // Get the configuration file path
        std::filesystem::path config_dir = curr_dir / "bt_client/config/tree.yaml";
        // Load the configuration file with the tree name
        YAML::Node config_file = YAML::LoadFile(config_dir);    

        // Get the directory of the behavior tree XML file using the name from the configuration file
        std::filesystem::path tree_dir = "bt_client/bt_xml/";
        xml_tree_path_ = tree_dir.string() + config_file["tree_name"].as<std::string>();

        flag_ = true;
    }

private:
    void checkFile()
    {
        if (rcpputils::fs::exists(xml_tree_path_) && flag_)
        {
            // Set the flag to false to avoid multiple launches of the behavior tree client
            flag_ = false;
            RCLCPP_INFO(this->get_logger(), "Behavior tree found. Launching behavior tree client...");
            // Launch the behavior tree client
            std::system("ros2 launch bt_client bt.launch.py");
            // Remove the behavior tree XML file
            std::remove(xml_tree_path_.c_str());
            // When the file is removed, set the flag to true and wait for the next file
            flag_ = true;
        }
    }

    rclcpp::TimerBase::SharedPtr timer_; // timer to check the file
    std::string xml_tree_path_; // full path to the XML file
    bool flag_; // flag to avoid multiple launches of the behavior tree client
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FileCheckerNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
