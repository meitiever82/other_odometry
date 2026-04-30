#!/usr/bin/env python3
"""
Mirror RGB image timestamps and publish D435i CameraInfo.
The bag has no /camera/color/camera_info, so we subscribe to
/camera/color/image_raw and publish a matching CameraInfo.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo

class CamInfoPub(Node):
    def __init__(self):
        super().__init__('cam_info_pub')
        # BEST_EFFORT for both (matches bag playback and rtabmap qos=2 setting)
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', qos)
        self.sub = self.create_subscription(Image, '/camera/color/image_raw', self.cb, qos)
        self.msg = CameraInfo()
        self.msg.header.frame_id = 'camera_color_optical_frame'
        self.msg.width = 640
        self.msg.height = 480
        self.msg.distortion_model = 'plumb_bob'
        self.msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        fx, fy, cx, cy = 607.58, 603.56, 335.82, 239.95
        self.msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self.msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.n = 0

    def cb(self, img_msg):
        self.msg.header.stamp = img_msg.header.stamp
        self.pub.publish(self.msg)
        self.n += 1
        if self.n % 100 == 1:
            self.get_logger().info(f'Published CameraInfo #{self.n}')

def main():
    rclpy.init()
    node = CamInfoPub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
