// Copyright 2022, Chang-Hong Chen
// All rights reserved.
//
// Author: Chang-Hong Chen
// Email: longhongc@gmail.com

#include <chrono>
#include <functional>
#include <memory>
#include <utility>

#include "ros2_turtlebot_practice/simple_walker.hpp"

using std::placeholders::_1;


enum class WALKER_STATE {INIT, FORWARD, BACK, TURN_RIGHT};

SimpleWalker::SimpleWalker()
: Node("simple_walker"), current_state_{WALKER_STATE::INIT}
{
  this->initialize();
}

void SimpleWalker::initialize()
{
  vel_publisher_ =
    this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  control_cycle_timer_ =
    this->create_wall_timer(
    100ms,
    std::bind(&SimpleWalker::controlCycleCallback, this)
    );

  scan_subscriber_ =
    this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    std::bind(&SimpleWalker::scanCallback, this, _1)
    );

  forward_velocity.linear.x = 0.2;
  backward_velocity.linear.x = -0.1;
  turn_velocity.angular.z = -0.2;

  // Start the robot with an angle
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "Start robot with an angle");
  auto turn_left_velocity = turn_velocity;
  turn_left_velocity.angular.z = 0.2;

  rclcpp::Time start_time = this->now();
  double elapsed_second = 0;
  while (elapsed_second < 3) {
    vel_publisher_->publish(turn_left_velocity);
    elapsed_second = (this->now() - start_time).seconds();
  }

  this->goState(WALKER_STATE::FORWARD);
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "Enter FORWARD state");
}

void SimpleWalker::controlCycleCallback()
{
  if (last_scan_ == nullptr) {
    return;
  }

  geometry_msgs::msg::Twist output_velocity;

  switch (current_state_) {
    case WALKER_STATE::FORWARD:
      output_velocity = forward_velocity;

      if (this->forward_to_back()) {
        this->goState(WALKER_STATE::BACK);
        RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Enter BACK state");
      }

      break;

    case WALKER_STATE::BACK:
      output_velocity = backward_velocity;

      if (this->back_to_turn()) {
        this->goState(WALKER_STATE::TURN_RIGHT);
        RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Enter TURN state");
      }

      break;

    case WALKER_STATE::TURN_RIGHT:
      output_velocity = turn_velocity;

      if (this->turn_to_forward()) {
        this->goState(WALKER_STATE::FORWARD);
        RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Enter FORWARD state");
      }

      break;

    default:
      break;
  }

  vel_publisher_->publish(output_velocity);
}

void SimpleWalker::scanCallback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  last_scan_ = std::move(msg);
}

void SimpleWalker::goState(WALKER_STATE new_state)
{
  this->current_state_ = new_state;
  this->state_start_time_ = this->now();
}

bool SimpleWalker::forward_to_back()
{
  bool blocked = false;
  // The scan range on both side from center
  int half_view_idxs = 70;  // degree
  int right_check_idx = 360 - half_view_idxs;
  int left_check_idx = 0 + half_view_idxs;
  // Check range on the right half
  for (int i = right_check_idx; i < 360; ++i) {
    if (last_scan_->ranges[i] < this->OBSTACLE_MARGIN) {
      blocked = true;
      break;
    }
  }

  // Check range on the left half
  for (int i = 0; i < left_check_idx; ++i) {
    if (last_scan_->ranges[i] < this->OBSTACLE_MARGIN) {
      blocked = true;
      break;
    }
  }

  return blocked;
}

bool SimpleWalker::back_to_turn()
{
  auto elapsed = this->now() - this->state_start_time_;
  return elapsed > this->BACK_TIME;
}

bool SimpleWalker::turn_to_forward()
{
  auto elapsed = this->now() - this->state_start_time_;
  return elapsed > this->TURN_TIME;
}
