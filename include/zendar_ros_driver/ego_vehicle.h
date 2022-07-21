#ifndef EGO_VEHICLE_H_
#define EGO_VEHICLE_H_
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <zendar/api/api.h>

namespace zen
{
    visualization_msgs::msg::Marker EgoVehicle();
} // namespace zen
#endif // EGO_VEHICLE_H_