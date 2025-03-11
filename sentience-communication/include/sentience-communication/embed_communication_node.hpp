#pragma once

#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>

class EmbedCommunicationNode : public rclcpp::Node {
private:
    boost::asio::io_context _ioContext;
    boost::asio::serial_port _serialPort;
    std::shared_ptr<rclcpp::TimerBase> _timer;

public:
    EmbedCommunicationNode();
    ~EmbedCommunicationNode() = default;

    void alive();

    void configureSerialPort(boost::asio::serial_port& port, std::string portname, uint16_t baudRate);
};
