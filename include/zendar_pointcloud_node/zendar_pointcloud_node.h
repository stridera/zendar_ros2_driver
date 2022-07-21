#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class ZenPointCloudNode : public rclcpp::Node
{
public:
    ZenPointCloudNode(int argc, char *argv[]);
    ~ZenPointCloudNode();

    void Process();

private:
    std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> _publishers;
    rclcpp::TimerBase::SharedPtr timer_;

    std::set<std::string> serials;
    const std::string url;
    const std::string topic_prefix = "zen/";
};