#pragma once
#include <chrono>
#include "behaviortree_cpp/behavior_tree.h"
#include "sonia_common_ros2/srv/dropper_service.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class DropperActionNode : public BT::SyncActionNode
{
public:
    DropperActionNode(const std::string &name, const BT::NodeConfig &config);
    ~DropperActionNode(){};

    static BT::PortsList providedPorts()
    {
        // This action has a single input port called "message"
        return {};
    }

    BT::NodeStatus tick() override
    {
        auto request = std::make_shared<sonia_common_ros2::srv::DropperService::Request>();
        request->side = sonia_common_ros2::srv::DropperService::Request::PORT_SIDE;

        while (!_dropper_client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result = _dropper_client->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(_internal_node, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

private:
    std::shared_ptr<rclcpp::Node> _internal_node;
    rclcpp::Client<sonia_common_ros2::srv::DropperService>::SharedPtr _dropper_client;
};
