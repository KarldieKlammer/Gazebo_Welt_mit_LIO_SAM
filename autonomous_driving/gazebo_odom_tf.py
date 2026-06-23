# #!/usr/bin/env python3
# """
# Publishes odom → chassis TF from Gazebo's AckermannSteering odometry.

# The AckermannSteering plugin integrates wheel velocities starting from
# (0, 0, 0) at simulation start, so the result is a proper relative odometry
# that maps cleanly to the odom → chassis transform.

# This node fills the TF gap before (and during) LIO-SAM initialization.
# Once LIO-SAM's TransformFusion is active it will overwrite this transform
# with its own estimate — TF2 always uses the most recently published value
# for a given child frame.

# Parameters
# ----------
# odom_topic  : str   Bridged ROS2 odometry topic (default: /model/prius_hybrid/odometry)
# odom_frame  : str   Parent TF frame (default: odom)
# base_frame  : str   Child  TF frame (default: chassis)
# """

# from geometry_msgs.msg import TransformStamped
# from nav_msgs.msg import Odometry
# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# import tf2_ros


# class GazeboOdomTf(Node):

#     def __init__(self):
#         super().__init__('gazebo_odom_tf')

#         self.declare_parameter('odom_topic', '/model/prius_hybrid/odometry')
#         self.declare_parameter('odom_frame', 'odom')
#         self.declare_parameter('base_frame', 'chassis')

#         odom_topic = self.get_parameter('odom_topic').value
#         self._odom_frame = self.get_parameter('odom_frame').value
#         self._base_frame = self.get_parameter('base_frame').value

#         qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=5,
#         )

#         self._br = tf2_ros.TransformBroadcaster(self)
#         self._sub = self.create_subscription(Odometry, odom_topic, self._cb, qos)

#         self.get_logger().info(
#             f'Publishing TF {self._odom_frame} → {self._base_frame} from {odom_topic}'
#         )

#     def _cb(self, msg: Odometry):
#         t = TransformStamped()
#         t.header.stamp = msg.header.stamp
#         t.header.frame_id = self._odom_frame
#         t.child_frame_id = self._base_frame
#         t.transform.translation.x = msg.pose.pose.position.x
#         t.transform.translation.y = msg.pose.pose.position.y
#         t.transform.translation.z = msg.pose.pose.position.z
#         t.transform.rotation = msg.pose.pose.orientation
#         self._br.sendTransform(t)


# def main(args=None):
#     rclpy.init(args=args)
#     node = GazeboOdomTf()
#     rclpy.spin(node)
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
