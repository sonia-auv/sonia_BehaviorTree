#include "sonia_BehaviorTree/MissionSwitchStatus.hpp"
using std::placeholders::_1;

MissionSwitchStatus::MissionSwitchStatus(const std::string &name, const BT::NodeConfig &config)
    : BT::ConditionNode(name, config), _mission_status(false)
{
    _internal_node = rclcpp::Node::make_shared("bt_mission_status");
    _mission_switch_sub = _internal_node->create_subscription<sonia_common_ros2::msg::MissionStatus>(
        "/provider_rs485/mission_status", 10, std::bind(&MissionSwitchStatus::updateStatus, this, _1));
}

void MissionSwitchStatus::updateStatus(const sonia_common_ros2::msg::MissionStatus &msg)
{
    _mission_status = msg.status;
}