#include <stdlib.h>
#include <string>
#include <fstream>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/loggers/bt_observer.h"
#include "sonia_BehaviorTree/DropperActionNode.hpp"
#include "sonia_BehaviorTree/MissionSwitchStatus.hpp"
#include "sonia_BehaviorTree/Approachobject.hpp"

using namespace BT;

int main(int argc, char const *argv[])
{
    if (argc < 2)
    {
        return EXIT_FAILURE;
    }
    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<MissionSwitchStatus>("MissionSwitchStatus");
    factory.registerNodeType<DropperActionNode>("DropperActionNode");

    std::string name = argv[1];
    std::string file_path = name + ".xml";

    // Get the directory containing the executable
    const char* homeDir = getenv("HOME");
    std::filesystem::path executablePath = std::string(homeDir) + "/ros2_sonia_ws/src/sonia_BehaviorTree/src/missions";

    std::filesystem::path relativeFilePath = file_path;

    std::filesystem::path fullFilePath = executablePath / relativeFilePath;

    rclcpp::init(argc, argv);
    auto tree = factory.createTreeFromFile(fullFilePath);

    // Helper function to print the tree.
    BT::printTreeRecursively(tree.rootNode());

    // The purpose of the observer is to save some statistics about the number of times
    // a certain node returns SUCCESS or FAILURE.
    // This is particularly useful to create unit tests and to check if
    // a certain set of transitions happened as expected
    BT::TreeObserver observer(tree);

    // Print the unique ID and the corresponding human readable path
    // Path is also expected to be unique.
    std::map<uint16_t, std::string> ordered_UID_to_path;
    for (const auto &[name, uid] : observer.pathToUID())
    {
        ordered_UID_to_path[uid] = name;
    }

    for (const auto &[uid, name] : ordered_UID_to_path)
    {
        std::cout << uid << " -> " << name << std::endl;
    }

    NodeStatus result = tree.tickWhileRunning(); 
    std::cout << "MISSION RESULT: " << result << std::endl;
    std::cout << "----------------" << std::endl;
    // print all the statistics
    for (const auto &[uid, name] : ordered_UID_to_path)
    {
        const auto &stats = observer.getStatistics(uid);

        std::cout << "[" << name
                  << "] \tT/S/F:  " << stats.transitions_count
                  << "/" << stats.success_count
                  << "/" << stats.failure_count
                  << std::endl;
    }

    return EXIT_SUCCESS;
}
