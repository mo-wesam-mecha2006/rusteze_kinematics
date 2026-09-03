"""
wheel_odometry_node.py

ROS2 node that:
  - subscribes to /encoder_speeds (std_msgs/msg/Float64MultiArray)
  - calls kinematics.forward(w) to get instantaneous chassis velocities
  - integrates (vx, vy, wz) over dt to update global pose (X, Y, theta)
  - publishes /odom (nav_msgs/msg/Odometry)
  - broadcasts the odom -> base_link TF

Owner: Member 5
"""

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

from rusteze_kinematics.robot_kinematics import build_kinematics


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class WheelOdometryNode(Node):

    def __init__(self):
        super().__init__('wheel_odometry_node')

        self.declare_parameter('drive_type', 'diff')
        self.declare_parameter('track_width', 0.30)
        self.declare_parameter('wheelbase', 0.30)
        self.declare_parameter('wheel_radius', 0.05)

        drive_type = self.get_parameter('drive_type').get_parameter_value().string_value
        L = self.get_parameter('track_width').get_parameter_value().double_value
        W = self.get_parameter('wheelbase').get_parameter_value().double_value
        R = self.get_parameter('wheel_radius').get_parameter_value().double_value

        self.kinematics = build_kinematics(drive_type, L, W, R)
        self.get_logger().info(f"Wheel odometry node started with drive_type='{drive_type}'")

        # Pose state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self._last_time = None

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.encoder_sub = self.create_subscription(
            Float64MultiArray, '/encoder_speeds', self.encoder_callback, 10
        )

    def encoder_callback(self, msg: Float64MultiArray):
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9

        if self._last_time is None:
            self._last_time = now_sec
            return  # need two samples to get a dt

        dt = now_sec - self._last_time
        self._last_time = now_sec
        if dt <= 0.0:
            return

        # TODO(Member 5): once forward() is implemented, this returns real
        # chassis velocities from the wheel encoder readings.
        vx, vy, wz = self.kinematics.forward(msg.data)

        # --- Pose integration (simple Euler integration in the odom frame) ---
        self.theta += wz * dt
        self.x += (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        self.y += (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt

        self._publish_odom(vx, vy, wz, now)
        self._broadcast_tf(now)

    def _publish_odom(self, vx, vy, wz, stamp):
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quaternion(self.theta)

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)

    def _broadcast_tf(self, stamp):
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = yaw_to_quaternion(self.theta)
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
