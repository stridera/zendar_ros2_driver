#pragma once

#include "zendar_ros_driver/publish.h"

#include <zendar/api/api.h>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rosgraph_msgs/msg/log.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <visualization_msgs/msg/marker.hpp>

/// \namespace -----------------------------------------------------------------
namespace zen
{

  class ZendarDriverNode
  {
  public:
    ZendarDriverNode(
        std::shared_ptr<ros::NodeHandle> node,
        const std::string &url,
        const float max_range,
        int argc,
        char *argv[]);

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
    std::shared_ptr<ros::NodeHandle> node;
    image_transport::ImageTransport image_transport{*this->node};

    RadarImagePublish<sensor_msgs::msgImageConstPtr> image_pub{"image_", this->image_transport};
    RadarPublish<sensor_msgs::msgPointCloud2> points_pub{"points_", *this->node};
    RadarPublish<geometry_msgs::msg::PoseStamped> points_metadata_pub{"points_metadata_", *this->node};

    ros::Publisher pose_pub = this->node->advertise<geometry_msgs::msg::PoseStamped>("/pose", 100);
    ros::Publisher logs_pub = this->node->advertise<rosgraph_msgs::msgLog>("/zpu_logs", 100);

    ros::Publisher pose_quality_pub = this->node->advertise<diagnostic_msgs::msgDiagnosticArray>("/diagnostics", 100);

    ros::Publisher occupancy_grid_pub = this->node->advertise<nav_msgs::msg::OccupancyGrid>("/occupancy_grid", 100);

    // Create range marker, and ego vehicle publisher as latched topics
    ros::Publisher range_markers_pub = this->node->advertise<nav_msgs::msg::OccupancyGrid>("/range_markers", 1, true);
    ros::Publisher ego_vehicle_pub =
        this->node->advertise<visualization_msgs::msgMarker>("/ego_vehicle", 1, true);

    tf2_ros::StaticTransformBroadcaster extrinsics_pub;
    tf2_ros::StaticTransformBroadcaster vehicle_to_map_pub;

    std::set<std::string> serials;
    const std::string url;
    const float max_range;
  };
} ///< \namespace zen
