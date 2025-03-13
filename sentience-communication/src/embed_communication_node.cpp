#include "rclcpp/rclcpp.hpp"
#include "../include/sentience-communication/embed_communication_node.hpp"
#include <array>
#include <cstddef>
#include <functional>

bool EmbedCommunicationNode::_checkHealth() {
    return this->_serialPort.is_open();
}

void EmbedCommunicationNode::_reconnect(const std::string& portName) {
    if (this->_serialPort.is_open()) {
        return;
    }
    this->_openSerialPort(portName);
}

void EmbedCommunicationNode::_openSerialPort(const std::string& portName) {
    boost::system::error_code ec;

    this->_serialPort.open(portName, ec);
    if (ec) {
        RCLCPP_ERROR(this->get_logger(), "Error opening serial port (%s), retrying in 3 seconds...", portName.c_str());
    } else {
        this->_serialPort.set_option(boost::asio::serial_port_base::baud_rate(9600));
    }
}

bool EmbedCommunicationNode::_detectPayload() {
    auto it = this->_buffer.begin();
    while (it != this->_buffer.end()) {
        it++;
    }
    return false;
}

void EmbedCommunicationNode::_read() {
    if (!this->_checkHealth()) {
        return;
    }
    boost::system::error_code ec;
    std::array<std::byte, 256> buffer;
    std::size_t len = this->_serialPort.read_some(boost::asio::buffer(buffer, 256), ec);

    if (ec) {
        RCLCPP_ERROR(this->get_logger(), "Error reading from serial port: %s",
                     ec.message().c_str());
    } else {
        this->_buffer.insert(this->_buffer.end(), buffer.begin(), buffer.begin() + len);
    }
}

EmbedCommunicationNode::EmbedCommunicationNode() :
    rclcpp::Node("embed_communication_node"),
    _ioContext(),
    _serialPort(this->_ioContext) {
        RCLCPP_INFO(this->get_logger(), "EmbedCommunicationNode created");

        this->_timer = create_wall_timer(std::chrono::seconds(1), std::bind(&EmbedCommunicationNode::alive, this));
    }

void EmbedCommunicationNode::alive() {
    RCLCPP_INFO(this->get_logger(), "Alive");
}

void EmbedCommunicationNode::configureSerialPort(boost::asio::serial_port& port, std::string portname, uint16_t baudRate) {
    port.open(portname);
    port.set_option(boost::asio::serial_port_base::baud_rate(baudRate));
}

int main(int ac, char **av) {
    rclcpp::init(ac, av);
    rclcpp::spin(std::make_shared<EmbedCommunicationNode>());
    rclcpp::shutdown();
    return 0;
}
