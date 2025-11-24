#include "rclcpp/rclcpp.hpp"
#include <message_filters/subscriber.h>
#include <ublox_msg_filters/exact_time.h>
#include <ublox_msgs/msg/nav_hpposllh.hpp>
#include <ublox_msgs/msg/nav_relposned9.hpp>
#include <ublox_msgs/msg/nav_velned.hpp>


class TestNode : public rclcpp::Node {
public:
  TestNode():
  Node("test_node"),
  qos_{HISTORY_DEPTH},
  sub1{this, "msg1", qos_.get_rmw_qos_profile()},
  sub2{this, "msg2", qos_.get_rmw_qos_profile()},
  sub3{this, "msg3", qos_.get_rmw_qos_profile()},
  sync_{MySyncPolicy{HISTORY_DEPTH}, sub1, sub2, sub3}
  {
    int extra_arg{42};
    using namespace std::placeholders;
    message_filters::Connection connection{sync_.registerCallback(std::bind(&TestNode::callback, this, _1, _2, _3, std::cref(extra_arg)))};
    RCLCPP_WARN(get_logger(), "Waiting for messages...");
    const auto start_time{now()};
    while ((now() - start_time).seconds() < 5.0)
    {
      rclcpp::spin_some(get_node_base_interface());
      extra_arg = std::round((now() - start_time).seconds());
    }
    RCLCPP_WARN(get_logger(), "Done for messages...");
    connection.disconnect();
    sync_.registerCallback(std::bind(&TestNode::callback, this, _1, _2, _3, std::cref(extra_arg)));
  }
  
  void callback
  (
    const ublox_msgs::msg::NavHPPOSLLH::ConstSharedPtr msg1,
    const ublox_msgs::msg::NavRELPOSNED9::ConstSharedPtr msg2,
    const ublox_msgs::msg::NavVELNED::ConstSharedPtr msg3,
    const int& extra_arg
  )
  {
    RCLCPP_INFO(get_logger(), "%d RX %u %u %u", extra_arg, msg1->i_tow, msg2->i_tow, msg3->i_tow);
  }
private:
  static constexpr size_t HISTORY_DEPTH{10};
  rclcpp::QoS qos_;
  message_filters::Subscriber<ublox_msgs::msg::NavHPPOSLLH> sub1;
  message_filters::Subscriber<ublox_msgs::msg::NavRELPOSNED9> sub2;
  message_filters::Subscriber<ublox_msgs::msg::NavVELNED> sub3;

  typedef ublox_msg_filters::ExactTime<
    ublox_msgs::msg::NavHPPOSLLH,
    ublox_msgs::msg::NavRELPOSNED9,
    ublox_msgs::msg::NavVELNED
  > MySyncPolicy;

  message_filters::Synchronizer<MySyncPolicy> sync_;
};


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node{std::make_shared<TestNode>()};
  RCLCPP_INFO_STREAM(node->get_logger(), "Ready to receive");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
