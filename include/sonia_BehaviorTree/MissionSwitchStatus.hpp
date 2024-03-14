#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_common_ros2/msg/mission_status.hpp"
#include "rclcpp/rclcpp.hpp"

class MissionSwitchStatus : public BT::ConditionNode
{
public:
    MissionSwitchStatus(const std::string &name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts()
    {
        // This action has a single input port called "message"
        return {};
    }

    BT::NodeStatus tick() override
    {
        rclcpp::spin_some(_internal_node);
        if (_mission_status)
        {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

private:
    void updateStatus(const sonia_common_ros2::msg::MissionStatus &msg);

    std::shared_ptr<rclcpp::Node> _internal_node;
    rclcpp::Subscription<sonia_common_ros2::msg::MissionStatus>::SharedPtr _mission_switch_sub;
    bool _mission_status;
};