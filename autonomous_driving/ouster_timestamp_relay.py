#!/usr/bin/env python3
"""
Adds a synthetic 't' field (uint32, nanoseconds relative to scan start)
to the Gazebo gpu_lidar PointCloud2, which ships without per-point
timestamps.  LIO-SAM's imageProjection requires this field: without it
timeScanEnd == timeScanCur, which causes deskewInfo() to reject every
scan with "Waiting for IMU data".

The timestamp is derived from the azimuth angle of each point:
  column = round((atan2(y, x) + pi) / (2*pi) * num_columns) % num_columns
  t [ns] = column * scan_period_ns / num_columns

Parameters
----------
scan_frequency : float  (default 10.0)  LiDAR rotation rate in Hz
num_columns    : int    (default 1024)  Azimuth samples per full revolution
input_topic    : str    (default /ouster/points)
output_topic   : str    (default /ouster/points_timestamped)
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField


class OusterTimestampRelay(Node):

    def __init__(self):
        super().__init__('ouster_timestamp_relay')

        self.declare_parameter('scan_frequency', 10.0)
        self.declare_parameter('num_columns', 1024)
        self.declare_parameter('input_topic', '/ouster/points')
        self.declare_parameter('output_topic', '/ouster/points_timestamped')

        freq = self.get_parameter('scan_frequency').value
        self._num_cols = int(self.get_parameter('num_columns').value)
        self._scan_period_ns = int(1e9 / freq)
        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._sub = self.create_subscription(PointCloud2, in_topic, self._cb, sub_qos)
        self._pub = self.create_publisher(PointCloud2, out_topic, pub_qos)

        self.get_logger().info(
            f'Relay {in_topic} → {out_topic}  '
            f'(scan_freq={freq} Hz, cols={self._num_cols})'
        )

    # ------------------------------------------------------------------

    def _cb(self, msg: PointCloud2):
        fields = {f.name: f for f in msg.fields}
        if 't' in fields:
            # Already has timestamps — just forward unchanged.
            self._pub.publish(msg)
            return

        ps = msg.point_step          # bytes per point (input)
        n = msg.width * msg.height   # total points

        if n == 0:
            return

        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, ps)

        x_off = fields['x'].offset
        y_off = fields['y'].offset

        x = np.frombuffer(raw[:, x_off:x_off + 4].copy().tobytes(), dtype=np.float32)
        y = np.frombuffer(raw[:, y_off:y_off + 4].copy().tobytes(), dtype=np.float32)

        # Azimuth angle → column index → relative timestamp in nanoseconds
        az = np.arctan2(y, x)                                           # [-π, π]
        col = np.floor(
            (az + math.pi) / (2.0 * math.pi) * self._num_cols
        ).astype(np.int64) % self._num_cols
        t_ns = (col * self._scan_period_ns // self._num_cols).astype(np.uint32)

        t_bytes = t_ns.view(np.uint8).reshape(n, 4)
        new_raw = np.concatenate([raw, t_bytes], axis=1)

        out = PointCloud2()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.is_bigendian = msg.is_bigendian
        out.is_dense = msg.is_dense
        out.point_step = ps + 4
        out.row_step = (ps + 4) * msg.width
        out.fields = list(msg.fields) + [
            PointField(name='t', offset=ps, datatype=PointField.UINT32, count=1)
        ]
        out.data = bytes(new_raw.tobytes())
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OusterTimestampRelay()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
