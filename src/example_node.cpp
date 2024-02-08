#include <rclcpp/rclcpp.hpp>

#include "event_camera_example/example.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  auto node = std::make_shared<event_camera_example::ExampleNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
