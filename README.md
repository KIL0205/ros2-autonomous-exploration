# Autonomous Exploration, Mapping, and Localization

ROS 2 software for autonomous TurtleBot exploration and navigation, developed for RBE 3002. The project combines SLAM Toolbox, a custom frontier explorer and risk-aware A* planner, and a custom path-following controller. The repository also contains Nav2 AMCL and a separate custom particle-filter localization implementation, but the included saved-map localization launch files need a valid map path before they can be run as a complete workflow.

## Features

- Frontier detection and autonomous exploration while SLAM Toolbox builds an occupancy map.
- C-space obstacle inflation and a risk map used by the custom A* planner.
- Custom `FollowPath` action controller with LiDAR-based emergency-stop behavior.
- Automatic return to the recorded starting pose when no frontiers remain; a nearby walkable target is chosen if home is unsafe.
- Gazebo Sim world and TurtleBot3 spawn launch.
- Nav2 AMCL and custom particle-filter localization source for saved-map experiments; see [Known limitations](#known-limitations).

## Requirements

- A Linux ROS 2 installation. The repository does not declare or test a specific ROS 2 distribution or Gazebo version; choose a distribution that provides the packages below and its compatible `ros_gz_sim`.
- ROS 2 build and runtime packages: `ament_cmake`, `rosidl_default_generators`, `rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `std_srvs`, `tf2_ros`, and `tf2_geometry_msgs`.
- Simulation and visualization: `ros_gz_sim`, `turtlebot3_gazebo`, `turtlebot3_description`, and `rviz2`.
- Navigation and mapping: `slam_toolbox`, `nav2_bringup`, `nav2_msgs`, `nav2_map_server`, `nav2_lifecycle_manager`, and `nav2_amcl`.
- Python modules used by the nodes: NumPy, SciPy, and OpenCV (`cv2`).
- For hardware, a TurtleBot3 with its matching `turtlebot3_bringup` package. The controller publishes `geometry_msgs/msg/TwistStamped` on `/cmd_vel`; confirm the installed robot driver subscribes to that message type before attempting motion.

The package manifests omit some dependencies used by the launch files and Python nodes. `rosdep` is still useful, but may not install everything listed above.

## Installation

The commands below assume ROS 2 and its package repositories are already installed. Replace `jazzy` with the ROS 2 distribution installed on your machine; this project does not identify a required distribution.

```bash
export ROS_DISTRO=jazzy
source /opt/ros/$ROS_DISTRO/setup.bash

# Run from this repository's root if it is not already inside ~/ros2_ws/src.
mkdir -p ~/ros2_ws/src
ln -s "$(pwd)" ~/ros2_ws/src/Autonomous_Explorer
cd ~/ros2_ws

# Install dependencies declared in package manifests.
rosdep install --from-paths src --ignore-src -r -y

# Install dependencies used by this project but not all declared in its manifests.
sudo apt update
sudo apt install \
  ros-${ROS_DISTRO}-ros-gz-sim \
  ros-${ROS_DISTRO}-turtlebot3-gazebo \
  ros-${ROS_DISTRO}-turtlebot3-description \
  ros-${ROS_DISTRO}-turtlebot3-bringup \
  ros-${ROS_DISTRO}-slam-toolbox \
  ros-${ROS_DISTRO}-nav2-bringup \
  ros-${ROS_DISTRO}-nav2-msgs \
  ros-${ROS_DISTRO}-nav2-map-server \
  ros-${ROS_DISTRO}-nav2-lifecycle-manager \
  ros-${ROS_DISTRO}-nav2-amcl \
  ros-${ROS_DISTRO}-tf2-geometry-msgs \
  ros-${ROS_DISTRO}-rosidl-default-generators \
  ros-${ROS_DISTRO}-rviz2 \
  python3-numpy python3-scipy python3-opencv

colcon build --symlink-install
source install/setup.bash
```

If an apt package is unavailable for the chosen distribution, use that distribution's ROS package index to resolve the matching package name and compatible Gazebo release. For TurtleBot3, set the model to the robot you actually use (the examples below use Burger):

```bash
export TURTLEBOT3_MODEL=burger
```

After opening a new terminal, source both the ROS underlay and workspace overlay again:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
```

## Usage

### Gazebo simulation: SLAM and autonomous exploration

In a sourced terminal, run the combined launch:

```bash
ros2 launch exploration slam_unified.launch.py sim:=true world:=small-field
```

This launch starts the repository's Gazebo Sim world, spawns a TurtleBot3, starts SLAM Toolbox, and launches the explorer, planner, controller, and RViz. The explorer selects frontier goals automatically; monitor the occupancy map on `/map` and the planner/explorer visualization topics in RViz. Do not start a second copy of these nodes while this launch is running.

The launch file references `lab4_config.rviz`, while the checked-in RViz file is named `lab4_config.rvis`. If the automatically launched RViz cannot load its configuration, start RViz with the file that is actually installed:

```bash
rviz2 -d "$(ros2 pkg prefix exploration)/share/exploration/rviz/lab4_config.rvis"
```

In RViz, use a goal tool publishing `geometry_msgs/msg/PoseStamped` on `/move_base_simple/goal` for a manual destination. Choose a free position on the current map. The planner sends the computed path to the controller's `follow_path` action. Avoid sending manual goals while autonomous exploration is active, since both workflows use the planner.

To publish a goal from the command line instead of RViz, replace the example coordinates with a known free map location:

```bash
ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 1.0}, orientation: {z: 0.0, w: 1.0}}}"
```

### Save a map and return home

When the explorer finds no remaining frontiers, it attempts to save the map and automatically plans a return to the stored starting pose. There is no separate, verified command-line trigger for return-home behavior.

The automatic save currently targets a developer-specific absolute directory in `explorer.py`, so do not rely on that save being available on another machine. To save the live map explicitly to a portable location, use Nav2's map saver while the map is being published:

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/explored_map
```

This should create `~/maps/explored_map.yaml` and its image file. Keep the YAML and image together; use the YAML file when configuring a map server.

### Physical TurtleBot: SLAM and exploration

These steps are static instructions only; hardware operation has not been tested here. Use a clear, supervised test area, keep an accessible physical/software stop available, and verify the robot's `/cmd_vel` subscription type before starting.

**On the robot**, configure the same ROS 2 distribution and `ROS_DOMAIN_ID` used by the development computer, then start the standard TurtleBot3 base bringup. `30` is an example domain ID; choose a value and use it on both machines:

```bash
export ROS_DOMAIN_ID=30
export TURTLEBOT3_MODEL=burger
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 launch turtlebot3_bringup robot.launch.py
```

**On the development computer**, set that same `ROS_DOMAIN_ID`, source the workspace, and start the project nodes without simulation:

```bash
export ROS_DOMAIN_ID=30
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch exploration slam_unified.launch.py sim:=false
```

The project launch starts SLAM Toolbox, exploration, planning, control, and RViz; it does not start the physical robot's base driver. Ensure both computers can discover each other through the ROS 2/DDS network. Configure the network's ROS 2 middleware or firewall as needed; this repository contains no robot IP or network configuration. The automatic map-save limitation above also applies to hardware.

### Saved-map localization and navigation

There is no ready-to-run saved-map AMCL workflow in the checked-in repository. The `exploration` AMCL launch files and `localization/mcl_unified.launch.py` refer to `explored_map.yaml`, which is not included. The custom `localization` package is a particle-filter node, not Nav2 AMCL. Create and configure a map and update the launch map path before using either localization workflow; do not assume these launches are ready for hardware or simulation.

If Nav2 AMCL has been separately configured with a valid map and is running, its global-relocalization services are:

```bash
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty "{}"
ros2 service call /request_nomotion_update std_srvs/srv/Empty "{}"
```

These services are provided by Nav2 AMCL, not by the custom particle-filter node. The repository's `blowup.py` helper calls these service names and then repeatedly requests no-motion updates, but its AMCL launch is subject to the missing-map limitation above.

## Useful ROS interfaces

| Interface | Type | Use |
|---|---|---|
| `/map` | `nav_msgs/msg/OccupancyGrid` | SLAM or map-server occupancy map |
| `/move_base_simple/goal` | `geometry_msgs/msg/PoseStamped` | RViz/manual navigation goal received by the planner |
| `/nav_path` | `nav_msgs/msg/Path` | Planned path visualization |
| `/follow_path` | `control/action/FollowPath` | Planner-to-controller path-following action |
| `/explorer/go_home` | `geometry_msgs/msg/PoseStamped` | Planner's home-goal input; no separate verified user trigger |
| `/map_saver/save_map` | `nav2_msgs/srv/SaveMap` | Nav2 map-saving service used by the explorer |
| `/reinitialize_global_localization` | `std_srvs/srv/Empty` | Nav2 AMCL global particle reset |
| `/request_nomotion_update` | `std_srvs/srv/Empty` | Nav2 AMCL no-motion update |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Velocity output from the custom controller |

## Troubleshooting

- **Package or executable not found:** source the ROS underlay and workspace overlay in every terminal; install dependencies missing from the package manifests.
- **No map or TF errors:** confirm the robot or simulation publishes `/scan` and `/odom`, use the expected `map`, `odom`, and `base_link` frames, and check that simulated nodes use simulation time.
- **Map absent in RViz:** check the `/map` publisher and RViz map QoS settings. The supplied launch also references an RViz filename that does not match the checked-in `.rvis` file.
- **Goal does not move the robot:** publish to `/move_base_simple/goal`, verify the planner and `follow_path` action server are running, and confirm the robot driver accepts `TwistStamped` on `/cmd_vel`.
- **AMCL map load fails:** the default AMCL/localization launches expect an `explored_map.yaml` that is not included; save a map and configure a valid map path first.

## Project report

See the [Autonomous Exploration, Mapping, and Localization project report](Autonimous_Exploration_Report.pdf) for the system architecture, methodology, implementation decisions, and results.

## Authors and acknowledgments

Developed as a team project for WPI RBE 3002. Refer to the project report for the team's attribution and project context.
