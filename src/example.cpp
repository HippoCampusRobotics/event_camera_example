#include "event_camera_example/example.hpp"

namespace event_camera_example {

ExampleNode::ExampleNode(const rclcpp::NodeOptions &_options)
    : Node("example_node", _options) {
  std::string topic;

  rclcpp::QoS qos_sub = rclcpp::SensorDataQoS().keep_last(1000);
  using event_camera_msgs::msg::EventPacket;
  topic = "event_camera/events";
  events_sub_ = create_subscription<EventPacket>(
      topic, qos_sub,
      [this](const EventPacket::SharedPtr msg) { OnEventPacket(msg); });

  event_updater_.SetEventPacketCallback(
      [this](std::unique_ptr<dv::EventPacket> event_packet) {
        OnConvertedEventPacket(std::move(event_packet));
      });
}

void ExampleNode::OnEventPacket(
    const event_camera_msgs::msg::EventPacket::SharedPtr _msg) {
  if (!decoder_) {
    decoder_ = decoder_factory_.newInstance(*_msg);
    RCLCPP_INFO(get_logger(), "Decoder created.");
  }
  event_updater_.time_base_ = _msg->time_base;
  decoder_->decode(&(_msg->events[0]), _msg->events.size(), &event_updater_);
}

void ExampleNode::OnConvertedEventPacket(
    std::unique_ptr<dv::EventPacket> _event_packet) {
  RCLCPP_INFO(get_logger(), "I have a dv::EventPacket with %zu events",
              _event_packet->elements.size());
}
}  // namespace event_camera_example
