cmake_minimum_required(VERSION 3.16)
project(bt_client)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(igus_rebel_commander REQUIRED)

include_directories(
    include
)

set(THIS_PACKAGE_DEPS 
    behaviortree_ros2
    std_msgs
    nav2_msgs
    btcpp_ros2_interfaces 
    yaml-cpp
    igus_rebel_commander
    ament_index_cpp
    )

######################################################
# build client
add_executable(client 
  src/main.cpp 
  src/MoveTo.cpp
  src/MoveToWithTimeout.cpp
  src/isGoalReachable.cpp
  src/GenerateNextDestination.cpp
  src/isExplorationComplete.cpp
  src/FollowAruco.cpp
  src/MoveManipulator.cpp
)
ament_target_dependencies(client ${THIS_PACKAGE_DEPS}) 
target_link_libraries(client ${YAML_CPP_LIBRARIES})

######################################################
# INSTALL

install(TARGETS
  client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    bt_xml
    config
    DESTINATION share/${PROJECT_NAME}
)


ament_export_dependencies(behaviortree_ros2 btcpp_ros2_interfaces)

ament_package()
