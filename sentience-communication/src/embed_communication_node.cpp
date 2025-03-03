#include "rclcpp/rclcpp.hpp"
#include "../include/sentience-communication/embed_communication_node.hpp"

EmbedCommunicationNode::EmbedCommunicationNode() : rclcpp::Node("embed_communication_node") {
    RCLCPP_INFO(this->get_logger(), "EmbedCommunicationNode created");

    this->_timer = create_wall_timer(std::chrono::seconds(1), std::bind(&EmbedCommunicationNode::alive, this));
}

void EmbedCommunicationNode::alive() {
    RCLCPP_INFO(this->get_logger(), "Alive");
}

int main(int ac, char **av) {
    rclcpp::init(ac, av);
    rclcpp::spin(std::make_shared<EmbedCommunicationNode>());
    rclcpp::shutdown();
    return 0;
}