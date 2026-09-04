"""
wheel_odometry_node.py

ROS2 node that:
  - subscribes to /encoder_speeds
  - converts wheel speeds to chassis velocity using forward kinematics
  - integrates velocity over time to estimate pose
  - publishes /odom
  - broadcasts odom -> base_link TF

Owner: Member 5
"""

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

from robot_kinematics import build_kinematics


def yaw_to_quaternion(yaw):
    """Convert planar yaw angle to quaternion."""

    q = Quaternion()

    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)

    return q


class WheelOdometryNode(Node):

    def __init__(self):

        super().__init__('wheel_odometry_node')

        # -----------------------------------------------------
        # ROS2 Parameters
        # -----------------------------------------------------

        self.declare_parameter('drive_type', 'diff')
        self.declare_parameter('track_width', 0.30)
        self.declare_parameter('wheelbase', 0.30)
        self.declare_parameter('wheel_radius', 0.05)

        drive_type = (
            self.get_parameter('drive_type')
            .get_parameter_value()
            .string_value
        )

        L = (
            self.get_parameter('track_width')
            .get_parameter_value()
            .double_value
        )

        W = (
            self.get_parameter('wheelbase')
            .get_parameter_value()
            .double_value
        )

        R = (
            self.get_parameter('wheel_radius')
            .get_parameter_value()
            .double_value
        )

        # -----------------------------------------------------
        # Build selected kinematics model
        # -----------------------------------------------------

        self.kinematics = build_kinematics(
            drive_type,
            L,
            W,
            R
        )

        self.drive_type = drive_type

        # -----------------------------------------------------
        # Robot pose
        # -----------------------------------------------------

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = None

        # -----------------------------------------------------
        # Publisher
        # -----------------------------------------------------

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        # -----------------------------------------------------
        # TF Broadcaster
        # -----------------------------------------------------

        self.tf_broadcaster = TransformBroadcaster(self)

        # -----------------------------------------------------
        # Encoder subscriber
        # -----------------------------------------------------

        self.encoder_sub = self.create_subscription(
            Float64MultiArray,
            '/encoder_speeds',
            self.encoder_callback,
            10
        )

        self.get_logger().info(
            f"Wheel odometry started "
            f"with drive_type='{drive_type}'"
        )

    # =========================================================
    # Encoder callback
    # =========================================================

    def encoder_callback(self, msg):

        wheel_speeds = list(msg.data)

        # -----------------------------------------------------
        # Basic input validation
        # -----------------------------------------------------

        if self.drive_type == 'omni3':

            if len(wheel_speeds) < 3:

                self.get_logger().warn(
                    'omni3 requires at least 3 encoder speeds'
                )

                return

        else:

            if len(wheel_speeds) < 4:

                self.get_logger().warn(
                    'Expected 4 encoder speeds'
                )

                return

        # -----------------------------------------------------
        # Calculate dt
        # -----------------------------------------------------

        current_time = self.get_clock().now()

        current_sec = current_time.nanoseconds * 1e-9

        if self.last_time is None:

            self.last_time = current_sec

            return

        dt = current_sec - self.last_time

        self.last_time = current_sec

        if dt <= 0.0:

            return

        # -----------------------------------------------------
        # Forward Kinematics
        #
        # wheel speeds
        #       ↓
        # vx, vy, wz
        # -----------------------------------------------------

        try:

            vx, vy, wz = self.kinematics.forward(
                wheel_speeds
            )

        except (NotImplementedError, ValueError, IndexError) as error:

            self.get_logger().warn(
                f'Forward kinematics failed: {error}'
            )

            return

        # -----------------------------------------------------
        # Convert robot-frame velocity to odom-frame velocity
        # -----------------------------------------------------

        theta_old = self.theta

        x_dot = (
            vx * math.cos(theta_old)
            -
            vy * math.sin(theta_old)
        )

        y_dot = (
            vx * math.sin(theta_old)
            +
            vy * math.cos(theta_old)
        )

        # -----------------------------------------------------
        # Integrate pose
        # -----------------------------------------------------

        self.x += x_dot * dt

        self.y += y_dot * dt

        self.theta += wz * dt

        # Keep theta bounded between -pi and pi
        self.theta = math.atan2(
            math.sin(self.theta),
            math.cos(self.theta)
        )

        # -----------------------------------------------------
        # Publish ROS odometry
        # -----------------------------------------------------

        self.publish_odometry(
            current_time,
            vx,
            vy,
            wz
        )

        # -----------------------------------------------------
        # Broadcast odom -> base_link
        # -----------------------------------------------------

        self.broadcast_tf(current_time)

    # =========================================================

    def publish_odometry(self, stamp, vx, vy, wz):

        odom = Odometry()

        # Frames
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = 'odom'

        odom.child_frame_id = 'base_link'

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation
        odom.pose.pose.orientation = (
            yaw_to_quaternion(self.theta)
        )

        # Robot velocity
        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.linear.y = float(vy)
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = float(wz)

        self.odom_pub.publish(odom)

    # =========================================================

    def broadcast_tf(self, stamp):

        transform = TransformStamped()

        transform.header.stamp = stamp.to_msg()

        transform.header.frame_id = 'odom'

        transform.child_frame_id = 'base_link'

        # Translation
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        # Rotation
        transform.transform.rotation = (
            yaw_to_quaternion(self.theta)
        )

        self.tf_broadcaster.sendTransform(transform)


# =============================================================
# Main
# =============================================================

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