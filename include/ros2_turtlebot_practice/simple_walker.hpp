// Copyright 2022, Chang-Hong Chen
// All rights reserved.
//
// Author: Chang-Hong Chen
// Email: longhongc@gmail.com

#ifndef ROS2_TURTLEBOT_PRACTICE__SIMPLE_WALKER_HPP_
#define ROS2_TURTLEBOT_PRACTICE__SIMPLE_WALKER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::chrono_literals::operator""s;
using std::chrono_literals::operator""ms;

/**
 * @Brief  Walker state
 */
enum class WALKER_STATE;

/**
 * @Brief  Simple walker behavior that avoid obstacles
 */
class SimpleWalker : public rclcpp::Node
{
public:
  /**
   * @Brief  Constructor
   */
  SimpleWalker();

private:
  /**
   * @Brief  Initialize ros functions
   */
  void initialize();

  /**
   * @Brief  Finite state machine for robot behavior
   */
  void controlCycleCallback();

  /**
   * @Brief  Callback function for laser scan subscriber
   *
   * @Param msg Laser scan message
   */
  void scanCallback(sensor_msgs::msg::LaserScan::UniquePtr msg);


  /**
   * @Brief  Transist state to another
   *
   * @Param new_state The next state
   */
  void goState(WALKER_STATE new_state);


  /**
   * @Brief  Condition from state Forward to Back
   *         If the robot is in range of obstacle avoidance,
   *         this function returns true.
   *
   * @Returns true to transist to Turn
   */
  bool forward_to_back();


  /**
   * @Brief  Condition from state Back to turn
   *         After several seconds in Back state,
   *         this function returns true.
   *
   * @Returns true to transist to Turn
   */
  bool back_to_turn();

  /**
   * @Brief  Condition from state Turn to Forward
   *         After several seconds in Turn state,
   *         this function returns true.
   *
   * @Returns True to transist to Forward
   */
  bool turn_to_forward();

  /**
   * @Brief  Publisher for robot veclocity
   */
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;

  /**
   * @Brief  Cycle timer for finite state machine
   */
  rclcpp::TimerBase::SharedPtr control_cycle_timer_;

  /**
   * @Brief  Subscriber for laser scan
   */
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;

  /**
   * @Brief  Robot velocity for Forward state
   */
  geometry_msgs::msg::Twist forward_velocity;

  /**
   * @Brief  Robot velocity for Backward state
   */
  geometry_msgs::msg::Twist backward_velocity;


  /**
   * @Brief  Robot velocity for Turn state
   */
  geometry_msgs::msg::Twist turn_velocity;


  /**
   * @Brief  The latest laser scan message
   */
  sensor_msgs::msg::LaserScan::UniquePtr last_scan_;

  /**
   * @Brief  Current robot state
   */
  WALKER_STATE current_state_;

  /**
   * @Brief  Start time for each state
   *         Used for measuring time spent in a state
   */
  rclcpp::Time state_start_time_;

  /**
   * @Brief  Duration in Back state
   */
  const rclcpp::Duration BACK_TIME{1s};

  /**
   * @Brief  Duration in Turn state
   */
  const rclcpp::Duration TURN_TIME{4s};

  /**
   * @Brief  The safety margin to obstacle
   */
  const float OBSTACLE_MARGIN{0.3};
};
#endif  // ROS2_TURTLEBOT_PRACTICE__SIMPLE_WALKER_HPP_
