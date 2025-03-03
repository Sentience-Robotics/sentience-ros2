#pragma once

#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>

class EmbedCommunicationNode : public rclcpp::Node {
private:
    std::shared_ptr<rclcpp::TimerBase> _timer;

public:
    EmbedCommunicationNode();
    ~EmbedCommunicationNode() = default;

    void alive();
};