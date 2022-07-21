#include "zendar_pointcloud_node.h"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ZenPointCloudNode>(argc, argv);
  rclcpp::spin(node);

  return 0;
}
