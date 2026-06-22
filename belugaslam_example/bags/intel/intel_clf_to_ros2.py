#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

from math import pi
import tf_transformations
from tf2_ros import StaticTransformBroadcaster

import time
from pathlib import Path


class IntelDatasetNode(Node):

    def __init__(self):
        super().__init__('intel_dataset_node')

        self.scan_pub = self.create_publisher(LaserScan, '/base_scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom_combined', 10)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 10)

        self.static_broadcaster = StaticTransformBroadcaster(self)

        script_dir = Path(__file__).resolve().parent
        self.file = open(script_dir / "intel.clf", "r")

        self.static_tf_sent = False
        self.last_stamp = None

        # timer rápido
        self.timer = self.create_timer(0.01, self.process_line)

    def process_line(self):

        line = self.file.readline()

        if not line:
            self.get_logger().info("Fin del dataset")
            self.timer.cancel()
            return

        tokens = line.strip().split()

        if len(tokens) < 2:
            return

        if tokens[0] != 'FLASER':
            return

        num_scans = int(tokens[1])

        if num_scans < 100:
            return

        # timestamp REAL del dataset
        laser_timestamp = float(tokens[-1])

        # reproducción temporal real
        if self.last_stamp is not None:

            dt = laser_timestamp - self.last_stamp

            if dt > 0 and dt < 1.0:
                time.sleep(dt)

        self.last_stamp = laser_timestamp

        # tiempo ROS
        t = rclpy.time.Time(
            seconds=laser_timestamp
        ).to_msg()

        # -------------------------
        # LaserScan
        # -------------------------

        ranges = []

        raw_ranges = tokens[2:2 + num_scans]

        for r in raw_ranges:
            val = float(r)
            ranges.append(val)

        # pose
        x = float(tokens[5 + num_scans])
        y = float(tokens[6 + num_scans])
        theta = float(tokens[7 + num_scans])

        scan = LaserScan()

        scan.header.stamp = t
        scan.header.frame_id = 'laser_link'

        scan.angle_min = -pi / 2
        scan.angle_max = pi / 2
        scan.angle_increment = pi / (num_scans - 1)

        scan.range_min = 0.1
        scan.range_max = 81.3

        scan.ranges = ranges

        self.scan_pub.publish(scan)

        # -------------------------
        # Odometry
        # -------------------------

        odom = Odometry()

        odom.header.stamp = t
        odom.header.frame_id = 'odom_combined'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y

        q = tf_transformations.quaternion_from_euler(
            0, 0, theta
        )

        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        self.odom_pub.publish(odom)

        # -------------------------
        # TF dinámica
        # -------------------------

        tf_msg = TFMessage()

        trans = TransformStamped()

        trans.header.stamp = t
        trans.header.frame_id = 'odom_combined'
        trans.child_frame_id = 'base_footprint'

        trans.transform.translation.x = x
        trans.transform.translation.y = y
        trans.transform.translation.z = 0.0

        trans.transform.rotation.x = q[0]
        trans.transform.rotation.y = q[1]
        trans.transform.rotation.z = q[2]
        trans.transform.rotation.w = q[3]

        tf_msg.transforms.append(trans)

        self.tf_pub.publish(tf_msg)

        # -------------------------
        # TF estática
        # -------------------------

        if not self.static_tf_sent:

            static_tf = TransformStamped()

            static_tf.header.stamp = t

            static_tf.header.frame_id = 'base_footprint'
            static_tf.child_frame_id = 'laser_link'

            static_tf.transform.translation.x = 0.0
            static_tf.transform.translation.y = 0.0
            static_tf.transform.translation.z = 0.0

            static_tf.transform.rotation.w = 1.0

            self.static_broadcaster.sendTransform(static_tf)

            self.static_tf_sent = True


def main():

    rclpy.init()

    node = IntelDatasetNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
