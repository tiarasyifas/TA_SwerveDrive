# RobotPadi (ROS 2 + Gazebo)

Simple guide for the latest code in this repository.

This workspace contains:
- `robotpadi_description`: robot model (URDF/Xacro), Gazebo worlds, launch files, controller config.
- `robotpadi_controller`: navigation, manual controllers, waypoint publisher, state monitor, and hardware serial bridge.

## 1) Quick Start (Simulation)

From workspace root (this folder):

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select robotpadi_description robotpadi_controller
source install/setup.bash
```

Run full swerve simulation:

```bash
ros2 launch robotpadi_description robot.launch.py
```

What starts in this launch:
- Gazebo + RViz
- Robot spawn + ros2_control controllers
- `swerve_nav_goal`
- `robot_state_monitor`
- `robotpadi_hardware_bridge`
- `paddy_waypoint_goal` (delayed start)

## 2) Which Launch File Should I Use?

### A. Full swerve stack (recommended)

```bash
ros2 launch robotpadi_description robot.launch.py
```

Use this when you want the main environment and full newest pipeline.
Default world: `paddy_generated_1020.world`.

### B. Differential drive variant

```bash
ros2 launch robotpadi_description robot_diff.launch.py
```

Uses differential goal follower (`diff_nav_goal`) and wheel control only.

### C. Minimal Gazebo + controllers (empty world)

```bash
ros2 launch robotpadi_description gazebo.launch.py
```

### D. RViz model view only (no Gazebo)

```bash
ros2 launch robotpadi_description display.launch.py
```

## 3) Main Controller Nodes (Newest)

Installed ROS 2 executables from `robotpadi_controller`:

```bash
ros2 run robotpadi_controller swerve_nav_goal
ros2 run robotpadi_controller diff_nav_goal
ros2 run robotpadi_controller diff_controller
ros2 run robotpadi_controller paddy_waypoint_goal
ros2 run robotpadi_controller robot_state_monitor
ros2 run robotpadi_controller robotpadi_hardware_bridge
```

What each does:
- `swerve_nav_goal`: follows `/goal_pose` with swerve commands.
- `diff_nav_goal`: follows `/goal_pose` with differential wheel behavior.
- `diff_controller`: manual differential controller + GUI.
- `paddy_waypoint_goal`: auto-publishes waypoint goals in a paddy-lane pattern.
- `robot_state_monitor`: republishes clean scalar topics for plotting/monitoring.
- `robotpadi_hardware_bridge`: bridges ROS commands to Arduino over serial and publishes `/joint_states` from feedback.

## 4) Topics You Will Use Most

Input goal:
- `/goal_pose` (`geometry_msgs/PoseStamped`)

Control outputs:
- `/wheel_controller/commands` (`std_msgs/Float64MultiArray`)
- `/steering_controller/commands` (`std_msgs/Float64MultiArray`)

State:
- `/odom`
- `/joint_states`
- `/tf`

Monitor topics (`robot_state_monitor`):
- `/robot_state/vx`, `/robot_state/vy`, `/robot_state/omega`
- `/motor_state/omega1..4`
- `/swerve_state/theta1..4`

## 5) Worlds

Available world files:
- `src/robotpadi_description/worlds/paddy_generated_1020.world`
- `src/robotpadi_description/worlds/paddy_generated_2040.world`
- `src/robotpadi_description/worlds/paddy_generated.world`
- `src/robotpadi_description/worlds/no_paddy.world`

Default world in full launch is currently `paddy_generated_1020.world`.

If you edit world generation script:

```bash
python3 src/robotpadi_description/worlds/generate_world.py > src/robotpadi_description/worlds/paddy_generated.world
```

Then rebuild:

```bash
colcon build --packages-select robotpadi_description
source install/setup.bash
```

## 6) Real Hardware Bridge (Arduino)

`robotpadi_hardware_bridge` defaults:
- serial port: `/dev/ttyUSB0`
- baud: `115200`

Run manually with custom port:

```bash
ros2 run robotpadi_controller robotpadi_hardware_bridge --ros-args -p serial_port:=/dev/ttyUSB1 -p serial_baud:=115200
```

If you are simulation-only and do not have a serial device, the bridge node may fail to connect. This is expected for hardware-disabled setups.

## 7) Common Issues

1. `package not found`
- Re-run build and source:
  ```bash
  colcon build --packages-select robotpadi_description robotpadi_controller
  source install/setup.bash
  ```

2. Robot not moving
- Check controllers:
  ```bash
  ros2 control list_controllers
  ```
- Confirm `/wheel_controller/commands` is being published.

3. RViz goal sent but no motion
- Ensure `/odom` is alive:
  ```bash
  ros2 topic echo /odom --once
  ```
- Confirm `swerve_nav_goal` or `diff_nav_goal` is running.

4. Serial errors from hardware bridge in simulation
- Ignore, or disable that node in `robot.launch.py` for simulation-only runs.

## 8) Repository Layout (Short)

```text
src/
  robotpadi_description/
    launch/
    urdf/
    config/
    worlds/
    models/
  robotpadi_controller/
    robotpadi_controller/
      swerve_nav_goal.py
      diff_nav_goal.py
      diff_controller.py
      paddy_waypoint_goal.py
      robot_state_monitor.py
      robotpadi_hardware_bridge.py
```

## 9) Notes

- This project currently keeps `TODO` fields for package license/description metadata.
- For consistent behavior, use the same ROS distro and Gazebo version across build and runtime.
