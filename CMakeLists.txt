cmake_minimum_required(VERSION 3.5)

project(behavior_tree_tutorial)

add_subdirectory(BehaviorTree.CPP)
find_package(BehaviorTreeV3)

add_executable(main main.cpp)
target_link_libraries(main PUBLIC BT::behaviortree_cpp_v3)
