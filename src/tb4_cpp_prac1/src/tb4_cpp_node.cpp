#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "irobot_create_msgs/msg/interface_buttons.hpp"
#include "irobot_create_msgs/msg/lightring_leds.hpp"

// int main(int argc, char ** argv)
// {
//   (void) argc;
//   (void) argv;

//   printf("hello world tb4_cpp_prac1 package\n");
//   return 0;
// }

class TurtleBot4Prac1 : public rclcpp::Node
{
public:
  TurtleBot4Prac1(): Node("tb4_prac1") //public constructor names the node as "tb4_prac1"
  {
    // Subscribe to the /interface_buttons topic
    interface_buttons_subscriber_ =
    this->create_subscription<irobot_create_msgs::msg::InterfaceButtons>(
    "/interface_buttons",
    rclcpp::SensorDataQoS(),
    std::bind(&TurtleBot4Prac1::interface_buttons_callback, this, std::placeholders::_1));
  }
private:
  // Interface buttons subscription callback
  void interface_buttons_callback(
    const irobot_create_msgs::msg::InterfaceButtons::SharedPtr create3_buttons_msg) // Button 1 is pressed
  {   
    if(create3_buttons_msg->button_1.is_pressed)
    {
      RCLCPP_INFO(this->get_logger(), "Button 1 Pressed!!!");
    }
  }
  // Interface Button Subscriber
  rclcpp::Subscription<irobot_create_msgs::msg::InterfaceButtons>::SharedPtr interface_buttons_subscriber_;
  // Lightring Publisher
  rclcpp::Publisher<irobot_create_msgs::msg::LightringLeds>::SharedPtr lightring_publisher_;

};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); // initialise ROS 2
  rclcpp::spin(std::make_shared<TurtleBot4Prac1>()); // starts processing data from the node, include subscriber callback
  rclcpp::shutdown();
  return 0;
}