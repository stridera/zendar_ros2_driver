#ifndef RANGE_MARKERS_H_
#define RANGE_MARKERS_H_
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <zendar/api/api.h>

namespace zen
{
    nav_msgs::msg::OccupancyGrid RangeMarkers(float max_range);
    cv::Mat CreateImg(const int &image_height,
                      const int &image_width,
                      const int &width,
                      const int &image_origin_x,
                      const int &image_origin_y,
                      const float &rounded_max_range,
                      const int &num_circles);
    nav_msgs::msg::OccupancyGrid CreateRosMessage(const int &image_width,
                                                  const int &image_height,
                                                  const float &resolution,
                                                  const float &origin_x,
                                                  const float &origin_y,
                                                  cv::Mat img);
} // namespace zen
#endif // RANGE_MARKERS_H_