#!/usr/bin/env python3
"""Nav2 (Jazzy) publishes geometry_msgs/Twist, but the TurtleBot3 Gazebo bridge
expects TwistStamped on /cmd_vel. This node converts /cmd_vel_nav -> /cmd_vel."""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist, TwistStamped


class Stamper(Node):
    def __init__(self):
        super().__init__('twist_stamper',
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.cb, 10)

    def cb(self, m):
        s = TwistStamped()
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id = 'base_link'
        s.twist = m
        self.pub.publish(s)


def main():
    rclpy.init()
    node = Stamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
