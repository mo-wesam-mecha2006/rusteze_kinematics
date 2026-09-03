# Shared Contract — Task 11.3

Everyone codes against this. If something needs to change, update this file
in the same PR so nobody merges against a stale contract.

## Class interface (robot_kinematics.py)

```python
Kinematics(L, W, R)          # L: track width/radius, W: wheelbase, R: wheel radius
    .inverse(vx, vy, wz) -> list[float]      # length 4, rad/s per wheel
    .forward(w: list[float]) -> (vx, vy, wz)
```

Subclasses: `DiffDriveKinematics`, `MecanumKinematics`,
`ThreeWheelOmniKinematics`, `FourWheelOmniKinematics` — all built through
`build_kinematics(drive_type, L, W, R)`.

Wheel order convention (confirm and fill in before merging):
`[front_left, front_right, rear_left, rear_right]` — TODO: Member 2/3 confirm
and update this line once matrices are defined.

## Topics

| Topic              | Type                             | Publisher        | Subscriber          |
|---------------------|-----------------------------------|-------------------|----------------------|
| `/cmd_vel`          | `geometry_msgs/msg/Twist`         | (test / teleop)   | `kinematics_node`    |
| `/wheel_setpoints`  | `std_msgs/msg/Float64MultiArray`  | `kinematics_node` | (motor driver)       |
| `/encoder_speeds`   | `std_msgs/msg/Float64MultiArray`  | (encoder driver)  | `wheel_odometry_node`|
| `/odom`             | `nav_msgs/msg/Odometry`           | `wheel_odometry_node` | (nav stack)       |

TF: `wheel_odometry_node` broadcasts `odom -> base_link`.

## ROS2 parameters (both nodes)

- `drive_type`: one of `diff`, `mecanum`, `omni3`, `omni4`
- `track_width` (L, meters)
- `wheelbase` (W, meters)
- `wheel_radius` (R, meters)

## Branches

- `main` — protected, merged via PR
- `kinematics-base` — Member 2 (base class + DiffDrive)
- `kinematics-omni` — Member 3 (Mecanum + 3-wheel + 4-wheel omni)
- `kinematics-node` — Member 4
- `wheel-odometry` — Member 5
