#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "irobot_create_msgs/msg/interface_buttons.hpp"
#include "irobot_create_msgs/msg/lightring_leds.hpp"

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world tb4_cpp_prac1 package\n");
  return 0;
}

class TurtleBot4Prac1 : public rclcpp::Node
{
public:
  TurtleBot4Prac1(): Node("tb4_prac1") //public constructor names the node as "tb4_prac1"
  {}
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); // initialise ROS 2
  rclcpp::spin(std::make_shared<TurtleBot4Prac1>()); // starts processing data from the node, include subscriber callback
  rclcpp::shutdown();
  return 0;
}