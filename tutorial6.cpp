#include <iostream>
#include <string>
#include <atomic>
#include <chrono>
#include <experimental/filesystem>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

using namespace BT;

void SleepMS(int ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

struct Pose2D {
  double x, y, theta;
};

namespace BT {
template <> Pose2D convertFromString(StringView str) {
  auto parts = splitString(str, ';');
  if (parts.size() != 3) {
    throw RuntimeError("invalid input");
  }
  Pose2D output;
  output.x = convertFromString<double>(parts[0]);
  output.y = convertFromString<double>(parts[1]);
  output.theta = convertFromString<double>(parts[2]);
  return output;
}
} // namespace BT

class MoveBaseAction : public AsyncActionNode {
public:
  MoveBaseAction(const std::string &name, const NodeConfiguration &config)
      : AsyncActionNode(name, config) {}

  static PortsList providedPorts() { return {InputPort<std::string>("goal")}; }
  NodeStatus tick() override {
    Pose2D goal;
    if (!getInput<Pose2D>("goal", goal)) {
      throw RuntimeError("missing required input [goal]");
    }
    std::cout << "[ MoveBase: STARTED]. goal: x = " << goal.x
              << " y = " << goal.y << " theta = " << goal.theta << std::endl;

    half_requested_.store(false);
    int count = 0;

    while (!half_requested_ && count++ < 25) {
      SleepMS(10);
    }
    std::cout << "[MoveBase: FINISHED]" << std::endl;

    return half_requested_ ? NodeStatus::FAILURE : NodeStatus::SUCCESS;
  }

private:
  std::atomic_bool half_requested_;
};

class SaySomething : public SyncActionNode {
public:
  SaySomething(const std::string &name, const NodeConfiguration &config)
      : SyncActionNode(name, config) {}
  static PortsList providedPorts() {
    return {InputPort<std::string>("message")};
  }

  NodeStatus tick() override {
    Optional<std::string> msg = getInput<std::string>("message");
    if (!msg) {
      throw RuntimeError("missing required input [message]", msg.error());
    }
    std::cout << "Robot says: " << msg.value() << std::endl;
    return NodeStatus::SUCCESS;
  }
};

int main(int argc, char **argv) {
  BehaviorTreeFactory factory;

  factory.registerNodeType<MoveBaseAction>("MoveBase");
  factory.registerNodeType<SaySomething>("SaySomething");

  if (argc < 2) {
    std::cout << "specify xml path." << std::endl;
    return 0;
  }
  std::string xml_path_arg = std::string(argv[1]);
  std::string xml_path_abs = xml_path_arg;
  auto cd = std::experimental::filesystem::current_path();
  if (xml_path_arg[0] != '/')
    xml_path_abs = std::string(cd.concat("/" + xml_path_arg));
  auto tree = factory.createTreeFromFile(xml_path_abs);

  NodeStatus status = tree.tickRoot();
  while (status == NodeStatus::RUNNING) {
    status = tree.tickRoot();
    SleepMS(1);
  }

  std::cout << "--------------" << std::endl;
  tree.blackboard_stack[0]->debugMessage();
  std::cout << "--------------" << std::endl;
  tree.blackboard_stack[1]->debugMessage();
  std::cout << "--------------" << std::endl;

  return 0;
}
