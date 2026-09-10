#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock as RclClock, ClockType

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock

from math import isfinite
import tf_transformations
from tf2_ros import StaticTransformBroadcaster

import time
from pathlib import Path
from carmen_reader import load_ordered_flaser, scan_angles


class IntelDatasetNode(Node):

    def __init__(self):
        super().__init__('intel_dataset_node')

        self.scan_pub = self.create_publisher(LaserScan, '/base_scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom_combined', 10)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 10)

        self.static_broadcaster = StaticTransformBroadcaster(self)

        script_dir = Path(__file__).resolve().parent
        self.declare_parameter('dataset_path', str(script_dir / 'intel.clf'))
        self.declare_parameter('replay_rate', 1.0)
        self.declare_parameter('start_delay', 2.0)
        self.declare_parameter('publish_clock', True)
        self.declare_parameter('laser_angle_min_deg', -90.0)
        self.declare_parameter('laser_angle_increment_deg', 1.0)
        self.angle_min_deg = float(self.get_parameter('laser_angle_min_deg').value)
        self.angle_increment_deg = float(self.get_parameter('laser_angle_increment_deg').value)
        scan_angles(180, self.angle_min_deg, self.angle_increment_deg)  # validate before replay
        self.replay_rate = float(self.get_parameter('replay_rate').value)
        delay = float(self.get_parameter('start_delay').value)
        if not isfinite(self.replay_rate) or self.replay_rate <= 0 or not isfinite(delay) or delay < 0:
            raise ValueError('replay_rate must be positive and start_delay nonnegative')
        self.records, backwards = load_ordered_flaser(self.get_parameter('dataset_path').value)
        self.get_logger().info(
            f'Loaded {len(self.records)} scans; sorted {backwards} backward timestamp transitions. '
            f'Replaying at {self.replay_rate:g}x with original acquisition stamps.')
        self.clock_pub = self.create_publisher(Clock, '/clock', 10) if self.get_parameter('publish_clock').value else None
        self.next_record = 0
        self.start_wall = time.monotonic() + delay
        self.start_stamp_ns = self.records[0].timestamp_ns
        self.static_tf_sent = False
        # A steady timer avoids deadlock when use_sim_time is enabled. No sleep in
        # callbacks, no skipped long gaps, and no per-scan accumulated sleep error.
        self.timer = self.create_timer(0.005, self.process_line, clock=RclClock(clock_type=ClockType.STEADY_TIME))

    def process_line(self):
        elapsed = time.monotonic() - self.start_wall
        if elapsed < 0:
            return
        replay_ns = min(self.records[-1].timestamp_ns,
                        self.start_stamp_ns + int(elapsed * self.replay_rate * 1_000_000_000))
        if self.clock_pub is not None:
            clock = Clock()
            clock.clock = rclpy.time.Time(nanoseconds=replay_ns).to_msg()
            self.clock_pub.publish(clock)
        if self.next_record == len(self.records):
            self.get_logger().info(f'Dataset complete: published {self.next_record} scans.')
            self.timer.cancel()
            return
        record = self.records[self.next_record]
        if record.timestamp_ns > replay_ns:
            return
        self.next_record += 1
        t = rclpy.time.Time(nanoseconds=record.timestamp_ns).to_msg()
        num_scans = len(record.ranges)
        ranges = list(record.ranges)
        x, y, theta = record.odometry

        scan = LaserScan()

        scan.header.stamp = t
        scan.header.frame_id = 'laser_link'

        # Intel's 180 returns use one-degree spacing (-90 through +89).
        # Do not stretch the array to include an extra, nonexistent +90-degree beam.
        scan.angle_min, scan.angle_max, scan.angle_increment = scan_angles(
            num_scans, self.angle_min_deg, self.angle_increment_deg)

        scan.range_min = 0.1
        scan.range_max = 81.3

        scan.ranges = ranges


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

        # Publish supporting transforms before the scan that requires them. The
        # consumer still resolves TF at the scan timestamp; DDS can reorder topics.
        self.scan_pub.publish(scan)


def main():

    rclpy.init()

    node = IntelDatasetNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
