#pragma once

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <zendar/api/api.h>

namespace zen {
visualization_msgs::MarkerArray Tracks(const zpb::drivable_area::Tracks& tracks);
visualization_msgs::Marker CreateEdgeMsg(const zpb::drivable_area::Track& track,
                                         int& edge_id);
visualization_msgs::Marker CreateVelocityMsg(const zpb::drivable_area::Track& track);
}  // namespace zen