#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TwistPublisher : public rclcpp::Node
{
public:
    TwistPublisher()
    : Node("twist_publisher"), linear_velocity_(0.0), angular_velocity_(0.5)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtlesim1/turtle1/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&TwistPublisher::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Publishing Twist messages");
    }

private:
    void timer_callback()
    {
        auto msg = geometry_msgs::msg::Twist();
        linear_velocity_ += 0.005;  // Incremento lineal por ciclo de temporizador
        msg.linear.x = linear_velocity_;
        msg.angular.z = angular_velocity_;
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing: linear=%.3f, angular=%.3f", linear_velocity_, angular_velocity_);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double linear_velocity_;
    const double angular_velocity_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwistPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}