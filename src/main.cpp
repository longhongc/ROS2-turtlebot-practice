// Copyright 2022, Chang-Hong Chen
// All rights reserved.
//
// Author: Chang-Hong Chen
// Email: longhongc@gmail.com

#include <memory>

#include "ros2_turtlebot_practice/simple_walker.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("rclcpp"),
    "Start turtlebot");

  rclcpp::spin(std::make_shared<SimpleWalker>());
  rclcpp::shutdown();

  return 0;
}
