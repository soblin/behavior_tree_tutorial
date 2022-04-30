#include <iostream>
#include <experimental/filesystem>
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

int main(int argc, char **argv) {
  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<ApproachObject>("ApproachObject");
  factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));
  factory.registerSimpleCondition("CheckTemperature",
                                  std::bind(CheckTemperature));
  GripperInterface gripper;
  factory.registerSimpleAction("OpenGripper",
                               std::bind(&GripperInterface::open, &gripper));
  factory.registerSimpleAction("CloseGripper",
                               std::bind(&GripperInterface::close, &gripper));
  std::string xml_path_abs = "./tutorial1.xml";
  if (argc >= 2) {
    std::string xml_path_arg = std::string(argv[1]);
    auto cd = std::experimental::filesystem::current_path();
    if (xml_path_arg[0] == '/') {
      xml_path_abs = xml_path_arg;
    } else {
      xml_path_abs = std::string(cd.concat("/" + xml_path_arg));
    }
  }
  auto tree = factory.createTreeFromFile(xml_path_abs);
  tree.tickRoot();
  return 0;
}
