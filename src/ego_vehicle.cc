#include "zendar_ros_driver/ego_vehicle.h"

#include <rclcpp/rclcpp.hpp>

namespace zen
{
  visualization_msgs::msg::Marker EgoVehicle()
  {
    visualization_msgs::msg::Marker ego_vehicle_msg;
    // Define header
    ego_vehicle_msg.header.frame_id = "map";
    ego_vehicle_msg.header.stamp = rclcpp::Clock().now();

    // Define type of marker
    ego_vehicle_msg.type = 1;
    ego_vehicle_msg.action = 0;
    ego_vehicle_msg.lifetime = rclcpp::Duration(0, 0);

    // Define scale
    ego_vehicle_msg.scale.x = 3;
    ego_vehicle_msg.scale.y = 2;
    ego_vehicle_msg.scale.z = 1.5;

    // Define color (corresponds to Zendar's green)
    ego_vehicle_msg.color.r = 0;
    ego_vehicle_msg.color.g = 186 / 255.0;
    ego_vehicle_msg.color.b = 95 / 255.0;
    ego_vehicle_msg.color.a = 1;

    // Define pose
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    ego_vehicle_msg.pose = pose;

    return ego_vehicle_msg;
  }
} // namespace zen