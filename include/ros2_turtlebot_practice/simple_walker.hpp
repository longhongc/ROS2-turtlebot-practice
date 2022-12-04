#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

enum class WALKER_STATE;

class SimpleWalker : public rclcpp::Node
{
public:
    SimpleWalker();

private:
    void initialize();
    void controlCycleCallback();
    void scanCallback(sensor_msgs::msg::LaserScan::UniquePtr msg);
    void goState(WALKER_STATE new_state);
    bool forward_to_back();
    bool back_to_turn();
    bool turn_to_forward();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;

    rclcpp::TimerBase::SharedPtr control_cycle_timer_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;

    geometry_msgs::msg::Twist forward_velocity;
    geometry_msgs::msg::Twist backward_velocity;
    geometry_msgs::msg::Twist turn_velocity;
    sensor_msgs::msg::LaserScan::UniquePtr last_scan_;

    WALKER_STATE current_state_;
    rclcpp::Time state_start_time_;
    const rclcpp::Duration BACK_TIME{1s};
    const rclcpp::Duration TURN_TIME{2s};

    const float OBSTACLE_MARGIN{0.5};
};
