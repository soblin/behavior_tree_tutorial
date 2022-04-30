#include <iostream>
#include <experimental/filesystem>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

class SaySomething : public BT::SyncActionNode {
public:
  SaySomething(const std::string &name, const BT::NodeConfiguration &config)
      : SyncActionNode(name, config) {}
  // declare which "port" is bound to this node in the xml
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("message")};
  }
  BT::NodeStatus tick() override {
    // get input "mesage" from the xml tag: <SaySomething message="hello"/>
    BT::Optional<std::string> msg = getInput<std::string>("message");
    if (!msg) {
      throw BT::RuntimeError("missing required input [message]: ", msg.error());
    }
    std::cout << "Robot says: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

BT::NodeStatus SaySomethingSimple(BT::TreeNode &self) {
  // get input from port "message"
  BT::Optional<std::string> msg = self.getInput<std::string>("message");
  if (!msg) {
    throw BT::RuntimeError("missing required input [message]: ", msg.error());
  }
  std::cout << "Robot says: " << msg.value() << std::endl;
  return BT::NodeStatus::SUCCESS;
}

class ThinkWhatToSay : public BT::SyncActionNode {
public:
  ThinkWhatToSay(const std::string &name, const BT::NodeConfiguration &config)
      : SyncActionNode(name, config) {}
  // bind "text" port to this node in the xml
  static BT::PortsList providedPorts() {
    return {BT::OutputPort<std::string>("text")};
  }
  BT::NodeStatus tick() override {
    setOutput("text", "The answer is 42");
    return BT::NodeStatus::SUCCESS;
  }
};

int main(int argc, char **argv) {
  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<SaySomething>("SaySomething");
  factory.registerNodeType<ThinkWhatToSay>("ThinkWhatToSay");

  // declare which "port" is bound to this node in the xml
  BT::PortsList say_something_ports = {BT::InputPort<std::string>("message")};
  factory.registerSimpleAction("SaySomething2", SaySomethingSimple,
                               say_something_ports);

  std::string xml_path_arg =
      (argc >= 2) ? std::string(argv[1]) : "./tutorial2.xml";
  auto cd = std::experimental::filesystem::current_path();
  std::string xml_path_abs = xml_path_arg;
  if (xml_path_arg[0] != '/')
    xml_path_abs = std::string(cd.concat("/" + xml_path_arg));
  // we do not need to manually create tree here.
  auto tree = factory.createTreeFromFile(xml_path_abs);
  tree.tickRoot();
  return 0;
}
