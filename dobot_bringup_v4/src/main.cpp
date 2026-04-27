

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/clock.hpp>
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <dobot_msgs_v4/msg/robot_status.hpp>
#include <dobot_msgs_v4/msg/tool_vector_actual.hpp>
#include <dobot_bringup/cr_robot_ros2.h>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto robot = std::make_shared<CRRobotRos2>();

  robot->init();

  rclcpp::spin(robot);

  // 关闭 ROS 2 环境
  rclcpp::shutdown();
  return 0;
}
