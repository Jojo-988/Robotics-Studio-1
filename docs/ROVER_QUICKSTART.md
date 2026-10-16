# Rover Simulation Quick Start

Launch the Husky rover with Gazebo, SLAM and the default Nav2 controller,
then send map coordinates for autonomous navigation.
Use **Ubuntu 22.04 + ROS 2 Humble + Ignition Gazebo Fortress**. Windows users
need WSL 2 with working WSLg graphics.

If ROS or Gazebo is not installed, complete the
[installation instructions in the README](../README.md) first.
Run the commands below in **Ubuntu Bash**, stopping if a command fails.

## 1. Select your workspace

On Windows, enter Ubuntu from PowerShell:

```powershell
wsl -d Ubuntu-22.04
```

Set your workspace and repository paths. Change these to match your checkout;
the demonstration computer uses `~/41068_github_ws` instead of `~/41068_ws`.

```bash
export ROVER_WS="$HOME/41068_ws"
export ROVER_REPO="$ROVER_WS/src/Robotics-Studio-1"
```

The team repository must already be at `ROVER_REPO`. Keep only one copy of
`41068_ignition_bringup` under the workspace's `src` directory. Build locally;
do not copy another computer's `build`, `install` or `log` directories.

## 2. Install project dependencies

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y git ros-dev-tools

# Initialize rosdep only if this computer has not been initialized.
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

cd "$ROVER_WS"
rosdep install --from-paths src --ignore-src --rosdistro humble -y
rosdep check --from-paths src --ignore-src --rosdistro humble
```

`rosdep` reads `package.xml` and installs the declared dependencies, including
the ROS/Gazebo bridge, SLAM, Nav2 and Python packages.

## 3. Build the rover package

```bash
cd "$ROVER_WS"
colcon build --symlink-install --packages-select 41068_ignition_bringup
source install/setup.bash
```

Repeat the dependency and build steps after relevant code updates.

## 4. Start the rover simulation

Run this block in a fresh Ubuntu terminal for each session. Set `ROVER_WS`
to the same workspace used above. Stop any previous simulation first.

```bash
export ROVER_WS="$HOME/41068_ws"
source /opt/ros/humble/setup.bash
source "$ROVER_WS/install/setup.bash"

export ROS_DOMAIN_ID=68
export ROS_LOCALHOST_ONLY=1
export LIBGL_ALWAYS_SOFTWARE=1
export QT_QPA_PLATFORM=xcb

ros2 launch 41068_ignition_bringup \
  41068_rover_navigation.launch.py world:=large_demo
```

Keep this terminal running. The forest world uses Gazebo Fuel models, so keep
internet access available and allow time for downloads on the first launch.
Gazebo opens with Husky, SLAM and navigation nodes.
**The rover waits for a goal and does not move automatically. RViz is not
opened by this launch.** Send a goal using Step 5 below.

Press **Ctrl+C in the launch terminal** to stop the simulation. Do not run
another Husky navigation stack or keyboard teleoperation alongside this mode.

## 5. Move the rover to map coordinates

Keep the simulation terminal running. Open a **second Ubuntu terminal**
(`wsl -d Ubuntu-22.04` from PowerShell), then load the same workspace and ROS
settings. Change `ROVER_WS` to match Step 4.

```bash
export ROVER_WS="$HOME/41068_ws"
source /opt/ros/humble/setup.bash
source "$ROVER_WS/install/setup.bash"
export ROS_DOMAIN_ID=68
export ROS_LOCALHOST_ONLY=1
```

Send a target position and heading to `/husky1/rover/goal`. This example requests
**x = 2 m, y = 0 m**, facing the map's positive X direction:

```bash
ros2 topic pub --once /husky1/rover/goal \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: husky1_map},
    pose: {
      position: {x: 2.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }}"
```

- Change `position.x` and `position.y` to your target, in metres. These are
  **absolute map coordinates**, not distances relative to the rover.
- Keep `position.z` at `0.0` and `header.frame_id` at `husky1_map`. Coordinates
  from Gazebo `world` or a drone map must be transformed into `husky1_map` first.
- Choose a reachable target in **known free space** in the current SLAM map.
  `(2, 0)` is a format example, not a guaranteed clear destination. This mode
  does not explore unknown areas automatically.
- `orientation` specifies the final heading as a quaternion. The example uses
  zero yaw. For +90 degrees, keep orientation x/y at zero and set orientation
  z/w to `0.70710678` each. For any yaw in radians, use `z = sin(yaw/2)` and
  `w = cos(yaw/2)`.

The task node validates the goal, requests an A* path from Nav2, and passes it
to the default DWB controller for automatic driving. A new valid goal replaces
the previous goal; wait for `SUCCEEDED` before sending the next waypoint if you
want a sequence. This dedicated mode uses `/husky1/rover/goal`, not RViz's
Nav2 Goal button.

Code entry: [rover_goal_navigation.py](../scripts/rover_goal_navigation.py).
Goal validation and navigation coordination:
[rover_navigation/node.py](../rover_navigation/node.py).

## 6. Monitor or cancel navigation

In the second terminal, monitor the task:

```bash
ros2 topic echo /husky1/rover/status std_msgs/msg/String
```

`FOLLOWING` means the rover is driving; `SUCCEEDED` confirms arrival within the
position and heading tolerances. For `WAITING` or `FAILED`, read `reason`;
`last_rejection` explains rejected goals. Publishing a goal is not confirmation
of arrival.

Pressing **Ctrl+C here stops monitoring only**. To cancel the goal and stop
the rover, exit monitoring and run:

```bash
ros2 service call /husky1/rover/cancel std_srvs/srv/Trigger "{}"
```

Confirm that the rover has stopped and status reports `CANCELED` with
`action_pending: false`. To end the entire simulation, press Ctrl+C in the
original launch terminal.

## Quick checks

- **Package or launch file not found:** confirm `ROVER_WS`, source its
  `install/setup.bash`, and rebuild if needed.
- **Missing trees or models:** check the launch log for Fuel download errors,
  confirm internet access, and allow model loading to finish.
- **No Gazebo window or OpenGL errors:** check WSLg and keep the two graphics
  environment variables shown above. Software rendering may be slower.
- **Other terminals cannot see the rover:** source the same workspace and use
  `ROS_DOMAIN_ID=68` and `ROS_LOCALHOST_ONLY=1` in those terminals too.
