#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <unordered_map>


/// \namespace -----------------------------------------------------------------
namespace zen {

template<typename Message>
class RadarPublish
{
public:
  RadarPublish(
    const std::string& topic_prefix,
    ros::NodeHandle& node,
    std::size_t backlog = 100
  )
    : topic_prefix(topic_prefix)
    , node(node)
    , backlog(backlog)
  { }

  void
  Publish(const std::string& serial, const Message& message)
  {
    if (this->publishers.find(serial) == this->publishers.end()) {
      std::string topic = this->topic_prefix + serial;
      this->publishers.emplace(
        std::make_pair(
          serial,
          this->node.advertise<Message>(topic, this->backlog)
      ));
    }

    this->publishers.at(serial).publish(message);
  }

private:
  const std::string topic_prefix;
  ros::NodeHandle& node;
  const std::size_t backlog;

  std::unordered_map<std::string, ros::Publisher> publishers;
};


template<typename Message>
class RadarImagePublish
{
public:
  RadarImagePublish(
    const std::string& topic_prefix,
    image_transport::ImageTransport& image_transport,
    std::size_t backlog = 100
  )
    : topic_prefix(topic_prefix)
    , image_transport(image_transport)
    , backlog(backlog)
  { }

  void
  Publish(const std::string& serial, const Message& message)
  {
    if (this->publishers.find(serial) == this->publishers.end()) {
      std::string topic = this->topic_prefix + serial;
      this->publishers.emplace(
        std::make_pair(
          serial,
          this->image_transport.advertise(topic, this->backlog)
      ));
    }

    this->publishers.at(serial).publish(message);
  }

private:
  const std::string topic_prefix;
  image_transport::ImageTransport image_transport;
  const std::size_t backlog;

  std::unordered_map<std::string, image_transport::Publisher> publishers;
};

} ///< \namespace
