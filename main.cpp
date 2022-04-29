#include <iostream>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

BT::NodeStatus CheckBattery() {
  std::cout << "[ Battery: OK ]" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CheckTemperature() {
  std::cout << "[ Temperature: OK ]" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SayHello() {
  std::cout << "Robot says: Hello World" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

class GripperInterface {
public:
  GripperInterface() : opened(false) {}
  BT::NodeStatus open() {
    opened = true;
    std::cout << "GripperInterface::open" << std::endl;
    return BT::NodeStatus::SUCCESS;
  };
  BT::NodeStatus close() {
    std::cout << "GripperInterface::close" << std::endl;
    opened = false;
    return BT::NodeStatus::SUCCESS;
  };

private:
  bool opened;
};

class ApproachObject : public BT::SyncActionNode {
public:
  ApproachObject(const std::string& name) : BT::SyncActionNode(name, {}) {}
  BT::NodeStatus tick() override {
    std::cout << "ApproachObject: " << this->name() << std::endl;
    return BT::NodeStatus::SUCCESS;
  };
};

class SaySomething : public BT::SyncActionNode {
public:
  SaySomething(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config) {}

  BT::NodeStatus tick() override {
    auto msg = getInput<std::string>("message");
    if (!msg) {
      throw BT::RuntimeError("missing required input [message]", msg.error());
    }
    std::cout << "Robot says: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
  };

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("message")};
  }
};

BT::NodeStatus SaySomethingSimple(BT::TreeNode &self) {
  auto msg = self.getInput<std::string>("message");
  if (!msg) {
    throw BT::RuntimeError("missing required input [message]", msg.error());
  }
  std::cout << "Robot says: " << msg.value() << std::endl;
  return BT::NodeStatus::SUCCESS;
};

void RegisterNodes(BT::BehaviorTreeFactory &factory) {
  static GripperInterface grip_singleton;

  factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));
  factory.registerSimpleCondition("CheckTemperature",
                                  std::bind(CheckTemperature));
  factory.registerSimpleAction("SayHello", std::bind(SayHello));
  factory.registerSimpleAction(
      "OpenGripper", std::bind(&GripperInterface::open, &grip_singleton));
  factory.registerSimpleAction(
      "CloseGrippe", std::bind(&GripperInterface::close, &grip_singleton));
  factory.registerNodeType<ApproachObject>("ApproachObject");
  factory.registerNodeType<SaySomething>("SaySomething");
}

BT_REGISTER_NODES(factory) { RegisterNodes(factory); }

int main() { return 0; }
