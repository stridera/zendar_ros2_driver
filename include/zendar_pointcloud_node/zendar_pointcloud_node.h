#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class ZenPointCloudNode : public rclcpp::Node
{
public:
    ZenPointCloudNode(int argc, char *argv[]);
    ~ZenPointCloudNode();

    void ProcessPoints();
    void ProcessLogs();

private:
    std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> _publishers;
    rclcpp::TimerBase::SharedPtr pointcloud_timer_;
    rclcpp::TimerBase::SharedPtr logs_timer_;

    std::set<std::string> serials;
    std::string url;
    const std::string topic_prefix = "zen/";
};