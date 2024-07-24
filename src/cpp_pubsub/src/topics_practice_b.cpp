#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

using namespace std::chrono_literals;

class TopicsPracticeB : public rclcpp::Node
{
public:
    TopicsPracticeB()
    : Node("topics_practice_b"), countdown_(5), in_spiral_mode_(true), linear_velocity_(0.9), angular_velocity_(1.1), current_x_(0.0), current_y_(0.0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&TopicsPracticeB::pose_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            1s, std::bind(&TopicsPracticeB::timer_callback, this));
    }

private:
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_x_ = msg->x;
        current_y_ = msg->y;
    }

    void timer_callback()
    {
        if (countdown_ > 0)
        {
            RCLCPP_INFO(this->get_logger(), "Application starts in: %d", countdown_);
            countdown_--;
        }
        else
        {
            if (in_spiral_mode_)
            {
                draw_spiral();
            }
            else
            {
                go_straight_or_avoid_walls();
            }
        }
    }

    void draw_spiral()
    {
        auto msg = geometry_msgs::msg::Twist();
        linear_velocity_ += 0.08;
        msg.linear.x = linear_velocity_;
        msg.angular.z = angular_velocity_;
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Drawing spiral");
        if (reached_boundaries())
        {
            in_spiral_mode_ = false;
        }
    }

    void go_straight_or_avoid_walls()
    {
        if (near_wall())
        {
            auto msg = geometry_msgs::msg::Twist();
            msg.linear.x = 0.0;
            msg.angular.z = 1.0;
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Avoiding walls");
        }
        else
        {
            auto msg = geometry_msgs::msg::Twist();
            msg.linear.x = linear_velocity_;
            msg.angular.z = 0.0;
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Going straight");
        }
    }

    bool near_wall()
    {
        const double x_limit = 1.0;
        const double y_limit = 1.0;
        return (current_x_ < x_limit || current_x_ > 10.5 - x_limit || current_y_ < y_limit || current_y_ > 10.5 - y_limit);
    }

    bool reached_boundaries()
    {
        const double x_limit = 9.0;
        const double y_limit = 9.0;
        return (current_x_ > x_limit || current_x_ < 1.0 || current_y_ > y_limit || current_y_ < 1.0);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
    int countdown_;
    bool in_spiral_mode_;
    double linear_velocity_;
    double angular_velocity_;
    double current_x_;
    double current_y_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto topics_practice_b = std::make_shared<TopicsPracticeB>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(topics_practice_b);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}