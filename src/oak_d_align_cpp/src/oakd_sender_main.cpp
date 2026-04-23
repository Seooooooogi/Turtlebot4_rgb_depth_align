#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "oak_d_align_cpp/oakd_sender.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<oak_d_align_cpp::OakdSender>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
