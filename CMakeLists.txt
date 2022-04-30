cmake_minimum_required(VERSION 3.5)

project(behavior_tree_tutorial)

add_subdirectory(BehaviorTree.CPP)
find_package(BehaviorTreeV3)

add_executable(tutorial1 tutorial1.cpp)
target_link_libraries(tutorial1 PUBLIC BT::behaviortree_cpp_v3 stdc++fs)

add_executable(tutorial2 tutorial2.cpp)
target_link_libraries(tutorial2 PUBLIC BT::behaviortree_cpp_v3 stdc++fs)
