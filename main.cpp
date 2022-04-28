#include <iostream>
#include <behaviortree_cpp_v3/behavior_tree.h>

class ApproachObject: public BT::SyncActionNode {
public:
  ApproachObject(const std::string& name) : BT::SyncActionNode(name, {}) {}

  BT::NodeStatus tick() override {
    std::cout << "ApproachObject: " << this->name() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

int main() {
  return 0;
}
