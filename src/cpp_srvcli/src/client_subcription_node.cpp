#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

class NodeSubscriberClient : public rclcpp::Node
{
public:
   NodeSubscriberClient() : Node("client_subscription_node_fail")
   {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&NodeSubscriberClient::listener_callback, this, std::placeholders::_1));

      client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
      while (!client_->wait_for_service(1s))
      {
            if (!rclcpp::ok())
            {
               RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
               rclcpp::shutdown();
               return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
      }
   }

   void send_request(int a, int b)
   {
      auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
      request->a = a;
      request->b = b;

      auto future = client_->async_send_request(request, std::bind(&NodeSubscriberClient::handle_add_two_ints_response, this, std::placeholders::_1));
   }

   void handle_add_two_ints_response(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future) {
      auto response = future.get();
      if (response) {
            RCLCPP_INFO(this->get_logger(), "The sum is: %ld", response->sum);
      } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service add_two_ints");
      }
   }

private:
   void listener_callback(const std_msgs::msg::String::SharedPtr msg)
   {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      if (msg->data == "Hello, world! 10")
      {
            RCLCPP_INFO(this->get_logger(), "Calling add_two_ints service...");
            send_request(2, 5);
      }
   }

   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
   rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char *argv[])
{
   rclcpp::init(argc, argv);
   auto sub_client_node = std::make_shared<NodeSubscriberClient>();
   rclcpp::spin(sub_client_node);
   rclcpp::shutdown();
   return 0;
}