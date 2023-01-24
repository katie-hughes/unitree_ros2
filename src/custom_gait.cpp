#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <cmath>

// Bad practice, fix
using namespace UNITREE_LEGGED_SDK;
using namespace std::chrono_literals;


class CustomGait : public rclcpp::Node
{
  public:
    CustomGait()
    : Node("custom_gait")
    {
      cmd_pub_ = this->create_publisher<ros2_unitree_legged_msgs::msg::LowCmd>("low_cmd", 10);
      timer_ = this->create_wall_timer(
        2ms,
        std::bind(&CustomGait::timer_callback, this));
      // initialize low_cmd fields
      low_cmd_ros.head[0] = 0xFE;
      low_cmd_ros.head[1] = 0xEF;
      low_cmd_ros.level_flag = LOWLEVEL;
      for (int i = 0; i < 12; i++)
      {
        low_cmd_ros.motor_cmd[i].mode = 0x0A;  // motor switch to servo (PMSM) mode
        low_cmd_ros.motor_cmd[i].q = PosStopF; // 禁止位置环
        low_cmd_ros.motor_cmd[i].kp = 0;
        low_cmd_ros.motor_cmd[i].dq = VelStopF; // 禁止速度环
        low_cmd_ros.motor_cmd[i].kd = 0;
        low_cmd_ros.motor_cmd[i].tau = 0;
      }
    }
  private:
    void timer_callback()
    {
      // RCLCPP_INFO_STREAM(get_logger(), "Timer tick!");
      if (!initiated_flag){
        RCLCPP_INFO_STREAM(get_logger(), "Waiting");
        count++;
        if (count>100){
          initiated_flag = true;
        }
      }else{
          motiontime += 2;
          RCLCPP_INFO_STREAM(get_logger(), "Initiated");
          double calf_1 = -M_PI / 2 + 0.5 * sin(2 * M_PI / 5.0 * motiontime * 1e-3);
          double calf_2 = -M_PI / 2 - 0.5 * sin(2 * M_PI / 5.0 * motiontime * 1e-3);
          double thigh_1 =  0.5 * sin(2 * M_PI / 5.0 * motiontime * 1e-3);
          double thigh_2 = -0.5 * sin(2 * M_PI / 5.0 * motiontime * 1e-3);
          double hip_1 = 0;// 0.5 * sin(2 * M_PI / 5.0 * motiontime * 1e-3);
          double hip_2 = 0;//-0.5 * sin(2 * M_PI / 5.0 * motiontime * 1e-3);
          low_cmd_ros.motor_cmd[FR_2].q = calf_1;
          low_cmd_ros.motor_cmd[FR_2].dq = 0.0;
          low_cmd_ros.motor_cmd[FR_2].kp = 5.0;
          low_cmd_ros.motor_cmd[FR_2].kd = 1.0;
          low_cmd_ros.motor_cmd[FR_0].q = hip_1;
          low_cmd_ros.motor_cmd[FR_0].dq = 0.0;
          low_cmd_ros.motor_cmd[FR_0].kp = 5.0;
          low_cmd_ros.motor_cmd[FR_0].kd = 1.0;
          low_cmd_ros.motor_cmd[FR_1].q = thigh_1;
          low_cmd_ros.motor_cmd[FR_1].dq = 0.0;
          low_cmd_ros.motor_cmd[FR_1].kp = 5.0;
          low_cmd_ros.motor_cmd[FR_1].kd = 1.0;

          low_cmd_ros.motor_cmd[FL_2].q = calf_2;
          low_cmd_ros.motor_cmd[FL_2].dq = 0.0;
          low_cmd_ros.motor_cmd[FL_2].kp = 5.0;
          low_cmd_ros.motor_cmd[FL_2].kd = 1.0;
          low_cmd_ros.motor_cmd[FL_0].q = hip_2;
          low_cmd_ros.motor_cmd[FL_0].dq = 0.0;
          low_cmd_ros.motor_cmd[FL_0].kp = 5.0;
          low_cmd_ros.motor_cmd[FL_0].kd = 1.0;
          low_cmd_ros.motor_cmd[FL_1].q = thigh_2;
          low_cmd_ros.motor_cmd[FL_1].dq = 0.0;
          low_cmd_ros.motor_cmd[FL_1].kp = 5.0;
          low_cmd_ros.motor_cmd[FL_1].kd = 1.0;

          low_cmd_ros.motor_cmd[RR_2].q = calf_2;
          low_cmd_ros.motor_cmd[RR_2].dq = 0.0;
          low_cmd_ros.motor_cmd[RR_2].kp = 5.0;
          low_cmd_ros.motor_cmd[RR_2].kd = 1.0;
          low_cmd_ros.motor_cmd[RR_0].q = hip_1;
          low_cmd_ros.motor_cmd[RR_0].dq = 0.0;
          low_cmd_ros.motor_cmd[RR_0].kp = 5.0;
          low_cmd_ros.motor_cmd[RR_0].kd = 1.0;
          low_cmd_ros.motor_cmd[RR_1].q = thigh_2;
          low_cmd_ros.motor_cmd[RR_1].dq = 0.0;
          low_cmd_ros.motor_cmd[RR_1].kp = 5.0;
          low_cmd_ros.motor_cmd[RR_1].kd = 1.0;

          low_cmd_ros.motor_cmd[RL_2].q = calf_1;
          low_cmd_ros.motor_cmd[RL_2].dq = 0.0;
          low_cmd_ros.motor_cmd[RL_2].kp = 5.0;
          low_cmd_ros.motor_cmd[RL_2].kd = 1.0;
          low_cmd_ros.motor_cmd[RL_0].q = hip_2;
          low_cmd_ros.motor_cmd[RL_0].dq = 0.0;
          low_cmd_ros.motor_cmd[RL_0].kp = 5.0;
          low_cmd_ros.motor_cmd[RL_0].kd = 1.0;
          low_cmd_ros.motor_cmd[RL_1].q = thigh_1;
          low_cmd_ros.motor_cmd[RL_1].dq = 0.0;
          low_cmd_ros.motor_cmd[RL_1].kp = 5.0;
          low_cmd_ros.motor_cmd[RL_1].kd = 1.0;
      }
      cmd_pub_->publish(low_cmd_ros);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    ros2_unitree_legged_msgs::msg::LowCmd low_cmd_ros;
    long motiontime = 0;
    bool initiated_flag = false;
    int count = 0;
    rclcpp::Publisher<ros2_unitree_legged_msgs::msg::LowCmd>::SharedPtr cmd_pub_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomGait>());
  rclcpp::shutdown();
  return 0;
}