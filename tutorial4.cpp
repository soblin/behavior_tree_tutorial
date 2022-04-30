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

NodeStatus CheckBattery() {
  std::cout << "[ Battery: OK ]" << std::endl;
  return NodeStatus::SUCCESS;
}

int main(int argc, char **argv) {
  BehaviorTreeFactory factory;

  factory.registerSimpleCondition("BatteryOK", std::bind(CheckBattery));
  factory.registerNodeType<MoveBaseAction>("MoveBase");
  factory.registerNodeType<SaySomething>("SaySomething");

  std::string xml_path_abs = "./tutorial4.xml";
  if (argc >= 2) {
    std::string xml_path_arg = std::string(argv[1]);
    auto cd = std::experimental::filesystem::current_path();
    if (xml_path_arg[0] == '/')
      xml_path_abs = xml_path_arg;
    else
      xml_path_abs = std::string(cd.concat("/" + xml_path_arg));
  } else {
    std::cout << "specify xml file." << std::endl;
    return 0;
  }
  auto tree = factory.createTreeFromFile(xml_path_abs);

  NodeStatus status;

  std::cout << "\n--- 1st executeTick() ---" << std::endl;
  status = tree.tickRoot();
  // At this point, the frist <SaySomething message="mission started..."/> is
  // executed, and finishes very shortly, so the status transits to next node,
  // "MoveBase". So [ MoveBase: STARTED]. goal: x = 1 y = 2 theta = 3 is also
  // printed very shortly. But its thread sleeps for 250ms.

  SleepMS(150);
  std::cout << "\n--- 2nd executeTick() ---" << std::endl;
  // So when we called tickRoot() for the second time, the status is still in
  // "MoveBase" node(because it hasn't returned NodeStatus yet!).
  status = tree.tickRoot();

  // during this SleepMS(150), "MoveBase" should have waited for 250ms, so
  // [MoveBase: FINISHED] is printed around here.
  SleepMS(150);
  // Now "MoveBase" should have already finished.

  std::cout << "\n--- 3rd executeTick() ---" << std::endl;
  status = tree.tickRoot();

  std::cout << std::endl;

  return 0;
}
