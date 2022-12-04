#include <chrono>
#include <functional>
#include <memory>

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
    return last_scan_->ranges[0] < this->OBSTACLE_MARGIN;
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
