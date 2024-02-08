#pragma once

#include <event_camera_codecs/decoder.h>
#include <event_camera_codecs/decoder_factory.h>

#include <dv_ros_msgs/msg/event_array.hpp>
#include <event_camera_msgs/msg/event_packet.hpp>
#include <rclcpp/rclcpp.hpp>

#include "event_updater.hpp"

namespace event_camera_example {

class ExampleNode : public rclcpp::Node {
 public:
  ExampleNode(const rclcpp::NodeOptions &_options);

 private:
  void OnEventPacket(const event_camera_msgs::msg::EventPacket::SharedPtr);
  void OnConvertedEventPacket(std::unique_ptr<dv::EventPacket>);

  EventUpdater event_updater_;
  std::shared_ptr<event_camera_codecs::Decoder<
      event_camera_msgs::msg::EventPacket, EventUpdater>>
      decoder_;
  event_camera_codecs::DecoderFactory<event_camera_codecs::EventPacket,
                                      EventUpdater>
      decoder_factory_;
  rclcpp::Subscription<event_camera_msgs::msg::EventPacket>::SharedPtr
      events_sub_;
};

}  // namespace event_camera_example
