#!/usr/bin/env python3
"""订阅 /leg_odometry，把位姿 dump 成 CSV。Ctrl-C 退出时 flush。

用法:
  ros2 run leg_odometry dump_leg_odom_csv.py [/output/path.csv]
默认输出路径: /tmp/leg_odom.csv
"""
import csv
import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class CSVDumper(Node):
    def __init__(self, path):
        super().__init__('leg_odom_csv_dumper')
        self.f = open(path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        self.n = 0
        self.sub = self.create_subscription(
            Odometry, '/leg_odometry', self.cb, 100)
        self.get_logger().info(f'writing to {path}')

    def cb(self, msg):
        h = msg.header.stamp
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.w.writerow([f'{h.sec + h.nanosec * 1e-9:.6f}',
                         f'{p.x:.6f}', f'{p.y:.6f}', f'{p.z:.6f}',
                         f'{q.x:.6f}', f'{q.y:.6f}', f'{q.z:.6f}', f'{q.w:.6f}'])
        self.n += 1
        if self.n % 1000 == 0:
            self.f.flush()
            self.get_logger().info(f'recorded {self.n} messages')

    def close(self):
        self.f.flush()
        self.f.close()
        self.get_logger().info(f'final: {self.n} messages')


def main():
    rclpy.init()
    path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/leg_odom.csv'
    node = CSVDumper(path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
