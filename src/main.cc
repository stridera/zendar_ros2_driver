#include "zendar_ros_driver/zendar_driver_node.h"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<zen::ZendarDriverNode>();
  rclcpp::spin(node);

  return 0;
}
