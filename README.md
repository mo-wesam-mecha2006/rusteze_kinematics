# rusteze_kinematics

Task 11.3 — Multi-Module Vehicle Convoy & Kinematics (MIA Robotics Electrical Team, Training 26/27).

A ROS2 package supporting four wheel configurations — Differential Drive,
Mecanum, 3-Wheel Omni, and 4-Wheel Omni — via a shared object-oriented
kinematics library, a kinematics node (chassis velocity -> wheel setpoints),
and a wheel odometry node (encoder feedback -> `/odom` + TF).

See [`CONTRACT.md`](./CONTRACT.md) for the shared topic/parameter/interface contract.

## Team

| Member | Responsibility |
|---|---|
| 1 | Repo, package setup, integration, README |
| 2 | `Kinematics` base class + `DiffDriveKinematics` |
| 3 | `MecanumKinematics`, `ThreeWheelOmniKinematics`, `FourWheelOmniKinematics` |
| 4 | `kinematics_node.py` |
| 5 | `wheel_odometry_node.py` |

## Build

```bash
cd ~/ros2_ws        # your workspace root, this package goes in ws/src/
colcon build --packages-select rusteze_kinematics
source install/setup.bash
```

## Run

```bash
ros2 launch rusteze_kinematics kinematics.launch.py drive_type:=diff
```

Or run nodes individually:

```bash
ros2 run rusteze_kinematics kinematics_node --ros-args -p drive_type:=diff
ros2 run rusteze_kinematics wheel_odometry_node --ros-args -p drive_type:=diff
```

## Test

```bash
# Send a velocity command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}"

# Watch the wheel setpoints it produces
ros2 topic echo /wheel_setpoints

# Fake encoder feedback to exercise the odometry node
ros2 topic pub /encoder_speeds std_msgs/msg/Float64MultiArray "{data: [1.0, 1.0, 1.0, 1.0]}"

# Watch odometry + TF
ros2 topic echo /odom
ros2 run tf2_ros tf2_echo odom base_link
```

## Status

- [ ] `DiffDriveKinematics` (Member 2)
- [ ] `MecanumKinematics`, `ThreeWheelOmniKinematics`, `FourWheelOmniKinematics` (Member 3)
- [ ] `kinematics_node.py` wired to real `inverse()` (Member 4)
- [ ] `wheel_odometry_node.py` wired to real `forward()` (Member 5)



How to run Task 11.3 



first Terminal :
cd ~/Downloads/rusteze_kinematics
source install/setup.bash
ros2 run rusteze_kinematics kinematics_node

second terminal : 
cd ~/Downloads/rusteze_kinematics
source install/setup.bash
ros2 run rusteze_kinematics wheel_odometry_node

third terminal : 
ros2 topic echo --flow-style /odom

fourth terminal : 
ros2 topic pub /encoder_speeds std_msgs/msg/Float64MultiArray "{data: [50.0, 50.0, 50.0, 50.0]}"

