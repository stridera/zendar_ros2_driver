#include "zendar_pointcloud_node.h"

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <gflags/gflags.h>

#include <zendar/api/api.h>

using namespace std::chrono_literals;

ZenPointCloudNode::ZenPointCloudNode(int argc, char *argv[]) : Node("zen_pointcloud_node")
{

    this->declare_parameter("url", std::string("169.254.0.150"));
    this->declare_parameter("max_range", 40.0);
    url = this->get_parameter("url").as_string();

    zen::api::ZenApi::Init(&argc, &argv);

    auto default_telem_ports = zen::api::ZenApi::TelemPortOptions();
    zen::api::ZenApi::Connect(url, default_telem_ports);

    auto default_data_ports = zen::api::ZenApi::DataPortOptions();
    zen::api::ZenApi::Bind(default_data_ports);

    // auto modes = zen::api::ZenApi::ListModes();
    // std::cout << "Available modes are: " << std::endl;
    // for (const auto &mode : modes)
    // {
    //     std::cout << "\t" << mode << std::endl;
    // }

    std::string FLAGS_imaging_mode = "evk_modes/ZenCropFieldOp";
    std::string FLAGS_stream_addr = "172.16.0.99";
    auto default_install_options = zen::api::ZenApi::InstallOptions();
    zen::api::ZenApi::Start(FLAGS_imaging_mode, default_install_options, FLAGS_stream_addr);

    zen::api::ZenApi::SubscribeTrackerStates();
    zen::api::ZenApi::SubscribeLogMessages();

    pointcloud_timer_ = this->create_wall_timer(1000ms, std::bind(&ZenPointCloudNode::ProcessPoints, this));
    logs_timer_ = this->create_wall_timer(100ms, std::bind(&ZenPointCloudNode::ProcessLogs, this));
}

ZenPointCloudNode::~ZenPointCloudNode()
{
    zen::api::ZenApi::UnsubscribeTrackerStates();
    zen::api::ZenApi::UnsubscribeLogMessages();
    zen::api::ZenApi::Release();
    zen::api::ZenApi::Disconnect();
}

void ZenPointCloudNode::ProcessPoints()
{
    while (auto points = zen::api::ZenApi::NextTrackerState(zen::api::ZenApi::Duration::zero()))
    {
        const auto &serial = points->meta().serial();
        auto detection = points->detection();
        std::cout << "Got points for serial " << serial << " with " << detection.size() << " points" << std::endl;

        auto msg = sensor_msgs::msg::PointCloud2();

        sensor_msgs::PointCloud2Modifier mod(msg);
        mod.setPointCloud2Fields(15,
                                 "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                 "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                 "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                 "x_ecef", 1, sensor_msgs::msg::PointField::FLOAT64,
                                 "y_ecef", 1, sensor_msgs::msg::PointField::FLOAT64,
                                 "z_ecef", 1, sensor_msgs::msg::PointField::FLOAT64,
                                 "mag", 1, sensor_msgs::msg::PointField::FLOAT32,
                                 "az_var", 1, sensor_msgs::msg::PointField::FLOAT32,
                                 "el_var", 1, sensor_msgs::msg::PointField::FLOAT32,
                                 "r", 1, sensor_msgs::msg::PointField::FLOAT32,
                                 "rad_vel", 1, sensor_msgs::msg::PointField::FLOAT32,
                                 "az", 1, sensor_msgs::msg::PointField::FLOAT32,
                                 "el", 1, sensor_msgs::msg::PointField::FLOAT32,
                                 "doa_snr_db", 1, sensor_msgs::msg::PointField::FLOAT32,
                                 "rd_mean_snr_db", 1, sensor_msgs::msg::PointField::FLOAT32);
        mod.resize(detection.size());
        msg.is_dense = true;

        sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
        sensor_msgs::PointCloud2Iterator<double> iter_x_ecef(msg, "x_ecef");
        sensor_msgs::PointCloud2Iterator<double> iter_y_ecef(msg, "y_ecef");
        sensor_msgs::PointCloud2Iterator<double> iter_z_ecef(msg, "z_ecef");
        sensor_msgs::PointCloud2Iterator<float> iter_mag(msg, "mag");
        sensor_msgs::PointCloud2Iterator<float> iter_az_var(msg, "az_var");
        sensor_msgs::PointCloud2Iterator<float> iter_el_var(msg, "el_var");
        sensor_msgs::PointCloud2Iterator<float> iter_r(msg, "r");
        sensor_msgs::PointCloud2Iterator<float> iter_rad_vel(msg, "rad_vel");
        sensor_msgs::PointCloud2Iterator<float> iter_az(msg, "az");
        sensor_msgs::PointCloud2Iterator<float> iter_el(msg, "el");
        sensor_msgs::PointCloud2Iterator<float> iter_doa_snr_db(msg, "doa_snr_db");
        sensor_msgs::PointCloud2Iterator<float> iter_rd_mean_snr_db(msg, "rd_mean_snr_db");

        for (auto point_data : detection)
        {
            float az = point_data.azimuth();
            float el = point_data.elevation();
            float r = point_data.range();

            *iter_x = std::sin(az) * std::cos(el) * r;
            ++iter_x;
            *iter_y = std::sin(el) * r;
            ++iter_y;
            *iter_z = std::cos(az) * std::cos(el) * r;
            ++iter_z;
            *iter_x_ecef = point_data.position().x();
            ++iter_x_ecef;
            *iter_y_ecef = point_data.position().y();
            ++iter_y_ecef;
            *iter_z_ecef = point_data.position().z();
            ++iter_z_ecef;
            *iter_mag = point_data.magnitude();
            ++iter_mag;
            *iter_az_var = point_data.azimuth_variance();
            ++iter_az_var;
            *iter_el_var = point_data.elevation_variance();
            ++iter_el_var;
            *iter_r = point_data.range();
            ++iter_r;
            *iter_rad_vel = point_data.range_velocity();
            ++iter_rad_vel;
            *iter_az = point_data.azimuth();
            ++iter_az;
            *iter_el = point_data.elevation();
            ++iter_el;
            *iter_doa_snr_db = point_data.doa_snr_db();
            ++iter_doa_snr_db;
            *iter_rd_mean_snr_db = point_data.rd_mean_snr_db();
            ++iter_rd_mean_snr_db;
        }

        if (this->_publishers.find(serial) == this->_publishers.end())
        {
            std::string topic = this->topic_prefix + serial;
            auto publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic, 100);
            this->_publishers.emplace(std::make_pair(serial, publisher));
        }
        this->_publishers.at(serial)->publish(msg);
    }
}

void ZenPointCloudNode::ProcessLogs()
{
    // Dump any pending log messages
    int i = 0;
    while (auto next_log = zen::api::ZenApi::NextLogMessage())
    {
        // std::cout << "LOG: " << next_log->canonical_message() << "\n";
        if (++i > 110)
        {
            break;
        }
    }
}
