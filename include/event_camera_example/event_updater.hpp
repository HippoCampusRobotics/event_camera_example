#include <event_camera_codecs/event_processor.h>

#include <dv-processing/core/core.hpp>
#include <rclcpp/logging.hpp>

namespace event_camera_example {

class EventUpdater : public event_camera_codecs::EventProcessor {
 public:
  /////////////////////////////////////////////////////////////////////////////
  /// Implementation of the required interface
  /////////////////////////////////////////////////////////////////////////////
  inline void eventCD(uint64_t t, uint16_t x, uint16_t y,
                      uint8_t polarity) override {
    if (!event_packet_) {
      event_packet_ = std::make_unique<dv::EventPacket>();
    }
    int64_t time = static_cast<int64_t>((time_base_ + t) / 1000);
    event_packet_->elements.emplace_back(time, x, y, polarity);
    if (!event_counter_) {
      RCLCPP_INFO(get_logger(),
                  "First event sensor time of current packet: %lu + %lu",
                  time_base_, t);
    }
    ++event_counter_;
  }
  void eventExtTrigger(uint64_t, uint8_t, uint8_t) override {}
  void finished() override {
    RCLCPP_INFO(get_logger(), "EventPacket message contained %lu events.",
                event_counter_);
    event_counter_ = 0;
    if (event_packet_callback_) {
      event_packet_callback_(GetEventPacket());
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  /// My custom example implementations
  /////////////////////////////////////////////////////////////////////////////
  void SetEventPacketCallback(
      std::function<void(std::unique_ptr<dv::EventPacket>)> fn) {
    event_packet_callback_ = fn;
  }
  inline std::unique_ptr<dv::EventPacket> GetEventPacket() {
    return std::move(event_packet_);
  }
  inline void ResetEventPacket() { event_packet_.reset(); }
  inline bool HasEventPacket() { return event_packet_ != nullptr; }

  void rawData(const char *, size_t) override {}

  uint64_t time_base_{0};

 private:
  static rclcpp::Logger get_logger() {
    return (rclcpp::get_logger("event_updater"));
  }

  std::function<void(std::unique_ptr<dv::EventPacket>)> event_packet_callback_;
  std::unique_ptr<dv::EventPacket> event_packet_;
  uint64_t event_counter_{0};
};

}  // namespace event_camera_example
