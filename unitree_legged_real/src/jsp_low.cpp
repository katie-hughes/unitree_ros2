#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

constexpr int NUM_MOTORS = 12;
constexpr int NUM_JOINTS = NUM_MOTORS + 1;

class JSPLowNode : public rclcpp::Node
{
public:
  JSPLowNode()
  : Node("jsp_low")
  {
    //Parameters
    auto param = rcl_interfaces::msg::ParameterDescriptor{};
    param.description = "The rate at which the node publishes joint states (Hz).";
    declare_parameter("rate", 100.0, param);
    rate_ = get_parameter("rate").get_parameter_value().get<double>();
    interval_ = 1.0 / rate_;

    //Timers
    timer_ = create_wall_timer(
      static_cast<std::chrono::microseconds>(static_cast<int>(interval_ * 1000000.0)),
      std::bind(&JSPLowNode::timer_callback, this)
    );

    //Publishers
    pub_joint_states_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    //Subscribers
    sub_state_ = create_subscription<ros2_unitree_legged_msgs::msg::LowState>(
      "low_state",
      10,
      std::bind(&JSPLowNode::state_callback, this, std::placeholders::_1)
    );

    sub_cmd_ = create_subscription<ros2_unitree_legged_msgs::msg::LowCmd>(
      "low_cmd",
      10,
      std::bind(&JSPLowNode::cmd_callback, this, std::placeholders::_1)
    );

    //Other variables
    joint_states_.name = {
      "FR_hip_joint",
      "FR_thigh_joint",
      "FR_calf_joint",
      "FL_hip_joint",
      "FL_thigh_joint",
      "FL_calf_joint",
      "RR_hip_joint",
      "RR_thigh_joint",
      "RR_calf_joint",
      "RL_hip_joint",
      "RL_thigh_joint",
      "RL_calf_joint",
      "base_to_footprint",
    };

    joint_states_.position = {};
    for (int i=0; i < NUM_JOINTS; i++) {
      joint_states_.position.push_back(0.0);
    }

  }
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<ros2_unitree_legged_msgs::msg::LowState>::SharedPtr sub_state_;
  rclcpp::Subscription<ros2_unitree_legged_msgs::msg::LowCmd>::SharedPtr sub_cmd_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;

  double rate_, interval_;
  ros2_unitree_legged_msgs::msg::LowState state_;
  ros2_unitree_legged_msgs::msg::LowCmd cmd_;
  sensor_msgs::msg::JointState joint_states_;

  void state_callback(const ros2_unitree_legged_msgs::msg::LowState::SharedPtr msg) {
    //Store most recent state for later use
    state_ = *msg;
  }

  void cmd_callback(const ros2_unitree_legged_msgs::msg::LowCmd::SharedPtr msg) {
    //Store most recent state for later use
    cmd_ = *msg;
  }

  void timer_callback()
  {
    //Update positions from most recent cmd
    for (int i=0; i < NUM_MOTORS; i++) {
      // joint_states_.position[i] = state_.motor_state[i].q;
      joint_states_.position[i] = cmd_.motor_cmd[i].q;
    }
    joint_states_.header.stamp = this->get_clock()->now();
    //Publish
    pub_joint_states_->publish(joint_states_);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JSPLowNode>());
  rclcpp::shutdown();
  return 0;
}
