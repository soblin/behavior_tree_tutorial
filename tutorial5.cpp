#include <iostream>
#include <experimental/filesystem>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_minitrace_logger.h>

using namespace BT;

namespace crossdoor {

static bool door_open = false;
static bool door_locked = true;

void SleepMS(int ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

NodeStatus IsDoorOpen() {
  std::cout << "Checking IsDoorOpen"
            << "\n";
  SleepMS(500);
  return door_open ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus IsDoorLocked() {
  std::cout << "Checking IsDoorLocked"
            << "\n";
  SleepMS(500);
  return door_locked ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus UnlockDoor() {
  if (door_locked) {
    std::cout << "Trying UnLockDoor"
              << "\n";
    SleepMS(2000);
    door_locked = false;
  }
  return NodeStatus::SUCCESS;
}

NodeStatus PassThroughDoor() {
  std::cout << "Trying PassThroughDoor"
            << "\n";
  SleepMS(1000);
  return door_open ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus PassThroughWindow() {
  std::cout << "Trying PassThroughWindow"
            << "\n";
  SleepMS(1000);
  return NodeStatus::SUCCESS;
}

NodeStatus OpenDoor() {
  if (door_locked) {
    std::cout << "Trying to OpenDoor, but the door is locked"
              << "\n";
    return NodeStatus::FAILURE;
  }
  std::cout << "Trying OpenDoor"
            << "\n";
  SleepMS(2000);
  door_open = true;
  return NodeStatus::SUCCESS;
}

NodeStatus CloseDoor() {
  if (door_open) {
    std::cout << "Trying CloseDoor"
              << "\n";
    SleepMS(1500);
    door_open = false;
  }
  return NodeStatus::SUCCESS;
}
} // namespace crossdoor

int main(int argc, char **argv) {
  using namespace crossdoor;

  if (argc < 2) {
    std::cout << "specify xml path"
              << "\n";
    return 0;
  }

  std::string xml_path_arg = std::string(argv[1]);
  auto cd = std::experimental::filesystem::current_path();
  std::string xml_path_abs = xml_path_arg;
  if (xml_path_arg[0] != '/')
    xml_path_abs = std::string(cd.concat("/" + xml_path_arg));

  BehaviorTreeFactory factory;

  factory.registerSimpleCondition("IsDoorOpen", std::bind(IsDoorOpen));
  factory.registerSimpleAction("PassThroughDoor", std::bind(PassThroughDoor));
  factory.registerSimpleAction("PassThroughWindow",
                               std::bind(PassThroughWindow));
  factory.registerSimpleAction("OpenDoor", std::bind(OpenDoor));
  factory.registerSimpleAction("CloseDoor", std::bind(CloseDoor));
  factory.registerSimpleCondition("IsDoorLocked", std::bind(IsDoorLocked));
  factory.registerSimpleAction("UnlockDoor", std::bind(UnlockDoor));

  auto tree = factory.createTreeFromFile(xml_path_abs);

  // StdCoutLogger logger_out(tree);
  // FileLogger logger_file(tree, "bt_trace.fbl");
  // MinitraceLogger logger_minitrace(tree, "bt_trace.json");
  // printTreeRecursively(tree.rootNode());

  NodeStatus status = NodeStatus::RUNNING;

  while (status == NodeStatus::RUNNING) {
    status = tree.tickRoot();
    SleepMS(1); // to avoid "busy loop"
  }

  return 0;
}
