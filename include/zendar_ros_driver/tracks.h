#pragma once

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <zendar/api/api.h>

namespace zen {
visualization_msgs::MarkerArray Tracks(const zpb::drivable_area::Tracks& tracks);
// TODO: Find out type of track
visualization_msgs::Marker CreateEdgeMsg(track,
                                         int& edge_id);
visualization_msgs::Marker CreateVelocityMsg(track)
}  // namespace zen