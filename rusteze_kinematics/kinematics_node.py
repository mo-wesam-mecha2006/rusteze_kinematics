"""
kinematics_node.py

ROS2 node that:
  - selects a drive kinematics class via parameters (drive_type, track_width,
    wheelbase, wheel_radius)
  - subscribes to /cmd_vel (geometry_msgs/msg/Twist)
  - calls kinematics.inverse(vx, vy, wz)
  - publishes the resulting wheel target speeds to /wheel_setpoints
    (std_msgs/msg/Float64MultiArray)

Owner: Member 4
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

from rusteze_kinematics.robot_kinematics import build_kinematics


class KinematicsNode(Node):

    def __init__(self):
        super().__init__('kinematics_node')

        self.declare_parameter('drive_type', 'diff')       # diff | mecanum | omni3 | omni4
        self.declare_parameter('track_width', 0.30)         # L
        self.declare_parameter('wheelbase', 0.30)            # W
        self.declare_parameter('wheel_radius', 0.05)         # R

        drive_type = self.get_parameter('drive_type').get_parameter_value().string_value
        L = self.get_parameter('track_width').get_parameter_value().double_value
        W = self.get_parameter('wheelbase').get_parameter_value().double_value
        R = self.get_parameter('wheel_radius').get_parameter_value().double_value

        self.kinematics = build_kinematics(drive_type, L, W, R)
        self.get_logger().info(f"Kinematics node started with drive_type='{drive_type}'")

        self.setpoint_pub = self.create_publisher(Float64MultiArray, '/wheel_setpoints', 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

    def cmd_vel_callback(self, msg: Twist):
       vx = msg.linear.x
       vy = msg.linear.y
       wz = msg.angular.z

       try:
            wheel_speeds = self.kinematics.inverse(vx, vy, wz)
       except NotImplementedError as e:
           self.get_logger().warn(str(e))
           return
       out = Float64MultiArray()
       out.data = [float(w) for w in wheel_speeds]
       self.setpoint_pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
