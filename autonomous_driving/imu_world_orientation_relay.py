#!/usr/bin/env python3
"""
Converts Gazebo IMU orientation from sensor-relative to world-frame.

gz-sim reports IMU orientation relative to the sensor's own initial pose
(identity at t=0), ignoring <localization>WORLD</localization> in some
versions.  LIO-SAM reads this quaternion in imuRPY2rosRPY() to initialize
the map heading — an identity quaternion makes it always start with yaw=0
regardless of the vehicle's actual spawn heading.

This node applies the vehicle's known spawn orientation as a fixed prefix
so that the published orientation represents the sensor pose in world frame:

    q_world = q_spawn ⊗ q_sensor

Parameters
----------
spawn_roll  : float  (default 0.0)  Vehicle spawn roll  [rad]
spawn_pitch : float  (default 0.0)  Vehicle spawn pitch [rad]
spawn_yaw   : float  (default 0.0)  Vehicle spawn yaw   [rad]
input_topic : str    (default /imu/data)
output_topic: str    (default /imu/data_world)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu


def _quat_from_rpy(roll, pitch, yaw):
    """RPY → [x, y, z, w] (ROS intrinsic XYZ = extrinsic ZYX)."""
    cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
    cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
    cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


def _quat_mul(q1, q2):
    """Hamilton product q1 ⊗ q2, both [x, y, z, w]."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ])


class ImuWorldOrientationRelay(Node):

    def __init__(self):
        super().__init__('imu_world_orientation_relay')

        self.declare_parameter('spawn_roll',  0.0)
        self.declare_parameter('spawn_pitch', 0.0)
        self.declare_parameter('spawn_yaw',   0.0)
        self.declare_parameter('input_topic',  '/imu/data')
        self.declare_parameter('output_topic', '/imu/data_world')

        roll  = self.get_parameter('spawn_roll').value
        pitch = self.get_parameter('spawn_pitch').value
        yaw   = self.get_parameter('spawn_yaw').value
        in_t  = self.get_parameter('input_topic').value
        out_t = self.get_parameter('output_topic').value

        self._q_spawn = _quat_from_rpy(roll, pitch, yaw)

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
        self._sub = self.create_subscription(Imu, in_t, self._cb, sub_qos)
        self._pub = self.create_publisher(Imu, out_t, pub_qos)

        self.get_logger().info(
            f'IMU world-orientation relay: {in_t} → {out_t}  '
            f'(spawn RPY: {roll:.4f}, {pitch:.4f}, {yaw:.4f} rad)'
        )

    def _cb(self, msg: Imu):
        q_sensor = np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ])

        q_world = _quat_mul(self._q_spawn, q_sensor)

        out = Imu()
        out.header = msg.header
        out.orientation.x = float(q_world[0])
        out.orientation.y = float(q_world[1])
        out.orientation.z = float(q_world[2])
        out.orientation.w = float(q_world[3])
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuWorldOrientationRelay()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
