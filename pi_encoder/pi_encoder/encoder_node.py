#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import pigpio

SIGNAL_A = 17
SIGNAL_B = 27

encoder_ticks = 0

class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_node')
        self.publisher_ = self.create_publisher(Int32, 'encoder_ticks', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.encoder_ticks_last = 0

        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error("Could not connect to pigpio daemon")
            exit(1)

        # Set up pins
        self.pi.set_mode(SIGNAL_A, pigpio.INPUT)
        self.pi.set_mode(SIGNAL_B, pigpio.INPUT)
        self.pi.set_pull_up_down(SIGNAL_A, pigpio.PUD_UP)
        self.pi.set_pull_up_down(SIGNAL_B, pigpio.PUD_UP)

        # Callback on rising edge
        self.pi.callback(SIGNAL_A, pigpio.RISING_EDGE, self.decode_encoder_ticks)
        self.get_logger().info("Encoder node initialized using pigpio")

    def decode_encoder_ticks(self, gpio, level, tick):
        global encoder_ticks
        if self.pi.read(SIGNAL_B) == 0:
            encoder_ticks -= 1
        else:
            encoder_ticks += 1

    def timer_callback(self):
        global encoder_ticks
        if encoder_ticks != self.encoder_ticks_last:
            msg = Int32()
            msg.data = encoder_ticks
            self.publisher_.publish(msg)
            self.get_logger().info(f"Encoder ticks: {encoder_ticks}")
            self.encoder_ticks_last = encoder_ticks

def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pi.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()