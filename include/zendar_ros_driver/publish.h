#pragma once

#include <rclcpp/rclcpp.hpp>

#include <unordered_map>

/// \namespace -----------------------------------------------------------------
namespace zen
{

  template <typename Message>
  class ZenPublish
  {
  public:
    ZenPublish(
        const std::string &topic_prefix,
        rclcpp::Node &node,
        std::size_t backlog = 100)
        : topic_prefix(topic_prefix), node(node), backlog(backlog)
    {
    }

    void
    Publish(const std::string &serial, const Message &message)
    {
      if (this->publishers.find(serial) == this->publishers.end())
      {
        std::string topic = this->topic_prefix + serial;
        auto publisher = this->node.create_publisher<Message>(topic, this->backlog);
        this->publishers.emplace(std::make_pair(serial, publisher));
      }

      this->publishers.at(serial).publish(message);
    }

  private:
    const std::string topic_prefix;
    rclcpp::Node &node;
    const std::size_t backlog;

    std::unordered_map<std::string, rclcpp::Publisher<Message>> publishers;
  };
} ///< \namespace
