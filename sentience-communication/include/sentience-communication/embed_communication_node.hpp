#pragma once

#include "rclcpp/rclcpp.hpp"
#include "boost/asio.hpp"
#include "boost/circular_buffer.hpp"
#include <iostream>
#include <typeinfo>
#include <queue>
#include <cstddef>

class EmbedCommunicationNode : public rclcpp::Node {
private:
    boost::asio::io_context _ioContext;
    boost::asio::serial_port _serialPort;
    boost::circular_buffer<std::byte> _buffer;
    std::shared_ptr<rclcpp::TimerBase> _timer;

    bool _checkHealth();
    void _reconnect(const std::string& portName);
    void _openSerialPort(const std::string& portName);

    bool _detectPayload();
    void _read();

public:
    EmbedCommunicationNode();
    ~EmbedCommunicationNode() = default;

    void alive();
};