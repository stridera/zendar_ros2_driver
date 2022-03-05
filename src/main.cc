#include "zendar_ros_driver/zendar_driver_node.h"

#include <ros/ros.h>


int main(int argc, char** argv) {
  ros::init(argc, argv, "zendar_driver_node");
  auto node = std::make_shared<ros::NodeHandle>("~");

  std::string url;
  if (!node->getParam("url", url)) {
    ROS_FATAL("IP address of ZPU was not provided");
  }

  zen::ZendarDriverNode converter(node, url, argc, argv);
  converter.Run();

  return 0;
}
