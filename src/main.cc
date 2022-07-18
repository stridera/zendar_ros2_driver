#include "zendar_ros_driver/zendar_driver_node.h"

#include <rcl_cpp/rcl_cpp.h>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<zen::ZendarDriverNode>();

  rclcpp::Parameter url = node->declare_parameter("url", std::string("192.168.1.9"));
  rclcpp::Parameter max_range = node->declare_parameter("max_range", std::float(40.0));
  zen::ZendarDriverNode converter(node, url.as_string(), max_range.as_float(), argc, argv);
  rclcpp::spin(node);

  return 0;
}
