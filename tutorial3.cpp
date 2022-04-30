#include <iostream>
#include <string>
#include <experimental/filesystem>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

using namespace BT;

struct Position2D {
  double x;
  double y;
};

namespace BT {
template <> Position2D convertFromString(StringView str) {
  std::cout << "Converting string \"" << str.data() << "\"\n";

  auto parts = splitString(str, ';');
  if (parts.size() != 2) {
    throw BT::RuntimeError("invalid input");
  } else {
    Position2D output;
    output.x = convertFromString<double>(parts[0]);
    output.y = convertFromString<double>(parts[1]);
    return output;
  }
}
} // namespace BT

class CalculateGoal : public SyncActionNode {
public:
  CalculateGoal(const std::string &name, const NodeConfiguration &config)
      : SyncActionNode(name, config) {}

  static PortsList providedPorts() { return {OutputPort<Position2D>("goal")}; }
  NodeStatus tick() override {
    Position2D myGoal = {1.1, 2.3};
    setOutput<Position2D>("goal", myGoal);
    return NodeStatus::SUCCESS;
  }
};

class PrintTarget : public SyncActionNode {
public:
  PrintTarget(const std::string &name, const NodeConfiguration &config)
      : SyncActionNode(name, config) {}
  static PortsList providedPorts() {
    const char *desc = "Simply print the goal on console...";
    return {InputPort<Position2D>("target", desc)};
  }
  NodeStatus tick() override {
    auto res = getInput<Position2D>("target");
    if (!res) {
      throw BT::RuntimeError("error reading port [target]: ", res.error());
    }
    Position2D target = res.value();
    std::cout << "Target positions: [" << target.x << ", " << target.y << "]\n";
    return NodeStatus::SUCCESS;
  }
};

int main(int argc, char **argv) {
  BehaviorTreeFactory factory;
  factory.registerNodeType<CalculateGoal>("CalculateGoal");
  factory.registerNodeType<PrintTarget>("PrintTarget");

  std::string xml_path_arg =
      (argc >= 2) ? std::string(argv[1]) : "./tutorial3.xml";
  auto cd = std::experimental::filesystem::current_path();
  std::string xml_path_abs = xml_path_arg;
  if (xml_path_arg[0] != '/')
    xml_path_abs = std::string(cd.concat("/" + xml_path_arg));
  auto tree = factory.createTreeFromFile(xml_path_abs);
  tree.tickRoot();
  return 0;
}
