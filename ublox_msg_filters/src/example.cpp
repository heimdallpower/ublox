#include "rclcpp/rclcpp.hpp"
#include <message_filters/subscriber.h>
#include <ublox_msg_filters/exact_time.h>
#include <ublox_msgs/msg/nav_hpposllh.hpp>
#include <ublox_msgs/msg/nav_relposned9.hpp>
#include <ublox_msgs/msg/nav_velned.hpp>

std::shared_ptr<rclcpp::Node> node;

void callback
(
  const ublox_msgs::msg::NavHPPOSLLH::ConstSharedPtr msg1,
  const ublox_msgs::msg::NavRELPOSNED9::ConstSharedPtr msg2,
  const ublox_msgs::msg::NavVELNED::ConstSharedPtr msg3
) {
  RCLCPP_INFO(node->get_logger(), "RX %u %u %u", msg1->i_tow, msg2->i_tow, msg3->i_tow);
}


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>("ublox_sync");
  
  constexpr size_t history_depth{10};
  rclcpp::QoS qos(history_depth);
  message_filters::Subscriber<ublox_msgs::msg::NavHPPOSLLH> sub1{node, "msg1", qos.get_rmw_qos_profile()};
  message_filters::Subscriber<ublox_msgs::msg::NavRELPOSNED9> sub2{node, "msg2", qos.get_rmw_qos_profile()};
  message_filters::Subscriber<ublox_msgs::msg::NavVELNED> sub3{node, "msg3", qos.get_rmw_qos_profile()};
  
  using namespace std::placeholders;
  typedef ublox_msg_filters::ExactTime<
  ublox_msgs::msg::NavHPPOSLLH,
  ublox_msgs::msg::NavRELPOSNED9,
  ublox_msgs::msg::NavVELNED
  > MySyncPolicy;

  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(history_depth), sub1, sub2, sub3);
  sync.registerCallback(callback);

  RCLCPP_INFO_STREAM(node->get_logger(), "Ready to receive");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
