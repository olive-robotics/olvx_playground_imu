#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"


#include "MainNode.hpp"

const auto launch_config_base_location = "/opt/olive/config/";

int main(int argc, char *argv[])
{
  std::string node_name = "app_position_control";
  std::cout << "Node name: " << node_name << std::endl;

  rclcpp::init(argc, argv);
  MainNode *node = new MainNode(launch_config_base_location,node_name);
  rclcpp::spin(node->ref_node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}