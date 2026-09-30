import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def finite_min(arr):
    vals = [x for x in arr if math.isfinite(x)]
    return min(vals) if vals else float('inf')


class ScanJudge(Node):
    def __init__(self):
        super().__init__('scan_judge')
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.safety_dist = 0.40
        self.last_print_t = self.get_clock().now()

    def scan_cb(self, msg: LaserScan):
        n = len(msg.ranges)
        if n < 30:
            return

        c = n // 2

        # 先用中间作为“前方”，两侧为左前/右前
        front = msg.ranges[max(0, c - 50): min(n, c + 50)]
        left  = msg.ranges[min(n, c + 40): min(n, c + 140)]
        right = msg.ranges[max(0, c - 140): max(0, c - 40)]

        front_min = finite_min(front)
        left_min = finite_min(left)
        right_min = finite_min(right)

        now = self.get_clock().now()
        if (now - self.last_print_t).nanoseconds > 2e8:  # 0.2 s 打印一次
            self.get_logger().info(
                f'front={front_min:.3f}, left={left_min:.3f}, right={right_min:.3f}'
            )
            self.last_print_t = now

        cmd = Twist()
        if front_min < self.safety_dist:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = 0.15
            cmd.angular.z = 0.0

        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = ScanJudge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()