#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/set_pen.hpp"

class ServicePracticeB : public rclcpp::Node {
public:
    ServicePracticeB() : Node("service_practice_b"), is_near_wall(false), color("green") {
        velocity_publisher = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        pose_subscriber = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&ServicePracticeB::pose_callback, this, std::placeholders::_1)
        );
        pen_service_client = this->create_client<turtlesim::srv::SetPen>("turtle1/set_pen");
    }

private:
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
        pose = *msg;
        check_boundaries();
        update_pen_color();
        move_turtle();
    }

    void check_boundaries() {
        if (pose.x < 1.0 || pose.x > 10.0 || pose.y < 1.0 || pose.y > 10.0) {
            is_near_wall = true;
        } else {
            is_near_wall = false;
        }
    }

    void update_pen_color() {
        if (pose.x > 5.5 && color != "red") {
            set_pen_color(255, 0, 0);  // Rojo
            color = "red";
            RCLCPP_INFO(this->get_logger(), "Set pen color to red");
        } else if (pose.x <= 5.5 && color != "green") {
            set_pen_color(0, 255, 0);  // Verde
            color = "green";
            RCLCPP_INFO(this->get_logger(), "Set pen color to green");
        }
    }

    void set_pen_color(uint8_t r, uint8_t g, uint8_t b) {
        if (pen_service_client->wait_for_service(std::chrono::seconds(1))) {
            auto req = std::make_shared<turtlesim::srv::SetPen::Request>();
            req->r = r;
            req->g = g;
            req->b = b;
            req->width = 3;  // Ancho del bolígrafo
            req->off = 0;  // Bolígrafo encendido
            pen_service_client->async_send_request(req);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service not available");
        }
    }

    void move_turtle() {
        auto vel_msg = geometry_msgs::msg::Twist();
        vel_msg.linear.x = is_near_wall ? 0.9 : 1.0;
        vel_msg.angular.z = is_near_wall ? 0.9 : 0.0;
        velocity_publisher->publish(vel_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_service_client;
    turtlesim::msg::Pose pose;
    bool is_near_wall;
    std::string color;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServicePracticeB>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}