#include "sonia_BehaviorTree/DropperActionNode.hpp"

DropperActionNode::DropperActionNode(const std::string &name, const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config)
{
    _internal_node = rclcpp::Node::make_shared("bt_dropper_action");
    _dropper_client = _internal_node->create_client<sonia_common_ros2::srv::DropperService>("actuate_dropper");
}