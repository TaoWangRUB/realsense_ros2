#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    // Create a subscription to the "/camera/pose/sample" topic
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/camera/pose/sample", 10, std::bind(&MinimalSubscriber::listener_callback, this, std::placeholders::_1));
  }

private:
  // Callback function for the subscription
  void listener_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received a message on /camera/pose/sample");
  }

  // Subscription object
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create an instance of the MinimalSubscriber node
  auto node = std::make_shared<MinimalSubscriber>();
  RCLCPP_INFO(node->get_logger(), "node initialized");
  // Spin the node to process incoming messages
  rclcpp::spin(node);

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}
