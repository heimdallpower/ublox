#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from ublox_msgs.msg import NavHPPOSLLH, NavRELPOSNED9, NavVELNED

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.itow = 0
        self.pub1 = self.create_publisher(NavHPPOSLLH, 'msg1', 2)
        self.pub2 = self.create_publisher(NavRELPOSNED9, 'msg2', 2)
        self.pub3 = self.create_publisher(NavVELNED, 'msg3', 2)
        self.timer = self.create_timer(0.2, self.publish)
        self.timer1 = self.create_timer(0.1, self.publish1)

    def publish1(self):
        self.get_logger().info("TX %u" % self.itow)
        msg1 = NavHPPOSLLH()
        msg1.i_tow = self.itow
        self.pub1.publish(msg1)
        self.itow += 1

    def publish(self):
        self.get_logger().info("TX %u" % self.itow)
        msg1 = NavHPPOSLLH()
        msg1.i_tow = self.itow
        self.pub1.publish(msg1)

        msg2 = NavRELPOSNED9()
        msg2.i_tow = self.itow
        self.pub2.publish(msg2)

        msg3 = NavVELNED()
        msg3.i_tow = self.itow
        self.pub3.publish(msg3)

        self.itow += 1

def main():
    rclpy.init()
    talker = Talker()
    talker.get_logger().info("Ready to transmit")
    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
