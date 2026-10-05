#include <chrono>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("tf_publisher_evidence");
  std::set<std::string> edges;
  auto callback = [&edges](tf2_msgs::msg::TFMessage::ConstSharedPtr msg,
                           const rclcpp::MessageInfo &info) {
    std::ostringstream gid;
    for (const auto byte : info.get_rmw_message_info().publisher_gid.data)
      gid << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
    for (const auto &tf : msg->transforms)
      edges.insert(tf.header.frame_id + " " + tf.child_frame_id + " " + gid.str());
  };
  auto dynamic = node->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", rclcpp::SensorDataQoS(), callback);
  auto fixed = node->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf_static", rclcpp::QoS(10).transient_local(), callback);
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(4);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < end) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  for (const auto &edge : edges) std::cout << edge << '\n';
  rclcpp::shutdown();
}
