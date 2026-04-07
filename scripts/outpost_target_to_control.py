#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from auto_aim_common.msg import Target
from gimbal_driver.msg import GimbalAngles

class Bridge(Node):
    def __init__(self):
        super().__init__('outpost_target_to_control_bridge')
        self.pub = self.create_publisher(GimbalAngles, '/ly/control/angles', 50)
        self.sub_target = self.create_subscription(Target, '/ly/outpost/target', self.cb_target, qos_profile_sensor_data)
        self.get_logger().info('bridge up: /ly/outpost/target -> /ly/control/angles (passthrough)')

    def cb_target(self, msg: Target):
        out = GimbalAngles()
        out.header = msg.header
        out.yaw = float(msg.yaw)
        out.pitch = float(msg.pitch)
        self.pub.publish(out)


def main():
    rclpy.init()
    n = Bridge()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
