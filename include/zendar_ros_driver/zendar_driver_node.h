#pragma once

#include "zendar_ros_driver/publish.h"

#include <zendar/api/api.h>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

/// \namespace -----------------------------------------------------------------
namespace zen
{

  class ZendarDriverNode : public rclcpp::Node
  {
  public:
    ZendarDriverNode();
    ~ZendarDriverNode();

    void
    Run();

  private:
    void ProcessImages();
    void ProcessPointClouds();
    void ProcessPoseMessages();
    void ProcessLogMessages();
    void ProcessHousekeepingReports();
    void ProcessRangeMarkers();
    void ProcessEgoVehicle();
    void ProcessOccupancyGrid();

    void ProcessHKGpsStatus(const zpb::telem::HousekeepingReport &report);
    void ProcessHKSensorIdentity(const zpb::telem::HousekeepingReport &report);
    void PublishExtrinsic(const zpb::telem::SensorIdentity &id);
    void PublishVehicleToMap();

  private:
    ZenPublish<sensor_msgs::msg::Image> image_pub;
    ZenPublish<sensor_msgs::msg::PointCloud2> points_pub;
    ZenPublish<geometry_msgs::msg::PoseStamped> points_metadata_pub;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped> pose_pub;
    rclcpp::Publisher<rcl_interfaces::msg::Log> logs_pub;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray> pose_quality_pub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid> occupancy_grid_pub;

    // Create range marker, and ego vehicle publisher as latched topics
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid> range_markers_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker> ego_vehicle_pub;

    tf2_ros::StaticTransformBroadcaster extrinsics_pub;
    tf2_ros::StaticTransformBroadcaster vehicle_to_map_pub;

    std::set<std::string> serials;
    const std::string url;
    const float max_range;
  };
} ///< \namespace zen
