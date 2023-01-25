#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

using UNITREE_LEGGED_SDK::UDP;
using UNITREE_LEGGED_SDK::HighCmd;
using UNITREE_LEGGED_SDK::HighState;

class UDPBridgeHigh
{
public:
    UDP udp;

    HighCmd cmd = {0};
    HighState state = {0};

public:
    UDPBridgeHigh()
        : udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState))
    {
        udp.InitCmdData(cmd);
    }
};

UDPBridgeHigh bridge;

rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr sub_high;
rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr pub_high;

long high_count = 0;

void highCmdCallback(const ros2_unitree_legged_msgs::msg::HighCmd::SharedPtr msg)
{
    printf("highCmdCallback is running !\t%ld\n", ::high_count);

    bridge.cmd = rosMsg2Cmd(msg);

    bridge.udp.SetSend(bridge.cmd);
    bridge.udp.Send();

    ros2_unitree_legged_msgs::msg::HighState state_ros;

    bridge.udp.Recv();
    bridge.udp.GetRecv(bridge.state);

    state_ros = state2rosMsg(bridge.state);

    pub_high->publish(state_ros);

    printf("highCmdCallback ending !\t%ld\n\n", ::high_count++);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("udp_high");

    printf("high level running!\n");

    pub_high = node->create_publisher<ros2_unitree_legged_msgs::msg::HighState>("high_state", 1);
    sub_high = node->create_subscription<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 1, highCmdCallback);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}