#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <memory>

class MinimalService : public rclcpp::Node
{
public:
  MinimalService()
  : Node("add_two_ints_server")
  {
    service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
      "add_two_ints", std::bind(&MinimalService::add, this, std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "Ready to add two ints.");
  }

private:
  void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
           std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
  {
    response->sum = request->a + request->b;
    RCLCPP_INFO(this->get_logger(), "Incoming request\na: %ld b: %ld", request->a, request->b);
    RCLCPP_INFO(this->get_logger(), "Sending back response: [%ld]", (long int)response->sum);
  }

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalService>());
  rclcpp::shutdown();
  return 0;
}