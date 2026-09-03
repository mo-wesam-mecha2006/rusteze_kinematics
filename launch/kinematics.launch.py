from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    drive_type_arg = DeclareLaunchArgument('drive_type', default_value='diff')
    track_width_arg = DeclareLaunchArgument('track_width', default_value='0.30')
    wheelbase_arg = DeclareLaunchArgument('wheelbase', default_value='0.30')
    wheel_radius_arg = DeclareLaunchArgument('wheel_radius', default_value='0.05')

    params = {
        'drive_type': LaunchConfiguration('drive_type'),
        'track_width': LaunchConfiguration('track_width'),
        'wheelbase': LaunchConfiguration('wheelbase'),
        'wheel_radius': LaunchConfiguration('wheel_radius'),
    }

    kinematics_node = Node(
        package='rusteze_kinematics',
        executable='kinematics_node',
        name='kinematics_node',
        parameters=[params],
        output='screen',
    )

    wheel_odometry_node = Node(
        package='rusteze_kinematics',
        executable='wheel_odometry_node',
        name='wheel_odometry_node',
        parameters=[params],
        output='screen',
    )

    return LaunchDescription([
        drive_type_arg,
        track_width_arg,
        wheelbase_arg,
        wheel_radius_arg,
        kinematics_node,
        wheel_odometry_node,
    ])
