# Drone Patrol — Team Installation and Startup Guide

These instructions cover the previous drone patrol version with startup checks,
sensor readiness checks, and standalone simulation mode. They do not include the
later model-download and resource-check scripts.

1. **Prepare the required environment.**

   Use the following versions:

   | Component | Required version |
   |---|---|
   | Ubuntu | 22.04 |
   | ROS 2 | Humble |
   | Gazebo | Ignition Fortress |
   | Windows users | WSL 2 with working Linux GUI support |

   Install any missing components using the official instructions:
   [ROS 2 Humble installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html),
   [Gazebo Fortress installation](https://gazebosim.org/docs/fortress/install_ubuntu/),
   and [Linux GUI applications in WSL](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps).

   **Run all commands below in an Ubuntu or WSL Bash terminal, not Windows PowerShell.**

2. **Install development tools and clone the repository.**

   After installing ROS 2 Humble and Gazebo Fortress:

   ```
   source /opt/ros/humble/setup.bash

   sudo apt update
   sudo apt install -y git ros-dev-tools

   mkdir -p ~/41068_ws/src
   cd ~/41068_ws/src

   git clone --branch main https://github.com/Jojo-988/Robotics-Studio-1.git
   ```

   The repository should now be located at:

   ```
   ~/41068_ws/src/Robotics-Studio-1
   ```

   For team testing, compare the commit ID:

   ```
   cd ~/41068_ws/src/Robotics-Studio-1
   git rev-parse HEAD
   git status --short
   ```

   Everyone should test the same commit. A clean checkout produces no output from
   `git status --short`. These instructions assume that `main` contains the patrol
   version agreed upon by the team; the branch name alone does not identify a version.

3. **Install project dependencies.**

   If rosdep has never been initialized on this computer, run this once:

   ```
   sudo rosdep init
   ```

   Skip that command if rosdep is already initialized. Then run:

   ```
   rosdep update

   source /opt/ros/humble/setup.bash
   cd ~/41068_ws

   rosdep install --from-paths src --ignore-src --rosdistro humble -y
   rosdep check --from-paths src --ignore-src --rosdistro humble
   ```

   Resolve any dependency errors before continuing.

4. **Build the project locally.**

   ```
   cd ~/41068_ws

   source /opt/ros/humble/setup.bash

   colcon build --symlink-install \
     --packages-select 41068_ignition_bringup

   source install/setup.bash
   ```

   Each teammate must build on their own computer. **Do not copy another person's
   `build/`, `install/`, or `log/` directories.**

5. **Configure communication and run the startup check.**

   ```
   cd ~/41068_ws/src/Robotics-Studio-1

   export ROS_LOCALHOST_ONLY=1
   export ROS_DOMAIN_ID=68

   bash scripts/start_coverage_patrol.sh --check
   ```

   `ROS_LOCALHOST_ONLY=1` keeps ROS communication on your own computer.
   `ROS_DOMAIN_ID=68` selects the communication domain used by this simulation.

   Teammates can all use domain 68 when running separate simulations with
   localhost-only communication enabled. Additional terminals monitoring the
   same simulation must use the same settings.

   The startup check verifies dependencies, the installed package, and environment
   settings. Actual sensor readiness is checked after the simulation starts.

6. **Start the drone simulation.**

   After the startup check passes:

   ```
   bash scripts/start_coverage_patrol.sh \
     mode:=standalone \
     rendering:=software
   ```

   The launcher loads the ROS/workspace environment, starts Gazebo and the drone
   controller, and creates an independent Gazebo partition. **You do not need to
   manually set `IGN_PARTITION`, `GZ_PARTITION`, or `LIBGL_ALWAYS_SOFTWARE`.**

   The expected sequence is:

   ```
   Wait for valid sensor data
          ↓
   Take off to approximately 15 metres
          ↓
   Follow the area-coverage patrol route
          ↓
   Return and hover above the starting position
   ```

   Return does **not** include automatic landing.

   The first launch needs internet access to download external scene models.
   Software rendering can improve graphics compatibility, but may run more slowly
   and still requires a working display/rendering environment.

   Stop the simulation with **Ctrl+C** before starting another instance. Do not
   run another drone velocity controller, keyboard teleoperation, or Parrot Nav2
   alongside this patrol.

7. **Use these commands for subsequent launches.**

   Once installation and compilation are complete:

   ```
   cd ~/41068_ws/src/Robotics-Studio-1

   export ROS_LOCALHOST_ONLY=1
   export ROS_DOMAIN_ID=68

   bash scripts/start_coverage_patrol.sh mode:=standalone rendering:=software
   ```

   Normal startup already runs the preflight checks, so a separate `--check`
   command is optional.

8. **Verify the simulation on your computer.**

   Stop the manual simulation first, then run the regression tests:

   ```
   cd ~/41068_ws

   source /opt/ros/humble/setup.bash
   source install/setup.bash

   colcon test --packages-select 41068_ignition_bringup
   colcon test-result --verbose
   ```

   To test a complete patrol automatically:

   ```
   cd ~/41068_ws/src/Robotics-Studio-1

   ROS_DOMAIN_ID=70 python3 test/validate_patrol_gazebo.py \
     --output ~/41068_ws/patrol_acceptance
   ```

   The validator starts and stops its own simulation without displaying the
   Gazebo window. Sensor rendering must still work.

   Check the result:

   ```
   cat ~/41068_ws/patrol_acceptance/result.json
   ```

   Acceptance requires:

   ```json
   "success": true
   ```

   If it fails, share your Git commit ID and these two files with the team:

   ```
   ~/41068_ws/patrol_acceptance/result.json
   ~/41068_ws/patrol_acceptance/gazebo_launch.log
   ```

9. **Rebuild after pulling project updates.**

   Stop the simulation and check for local changes:

   ```
   cd ~/41068_ws/src/Robotics-Studio-1
   git status --short
   ```

   Preserve any local changes before pulling. With a clean checkout on `main`:

   ```
   git pull --ff-only origin main

   cd ~/41068_ws
   source /opt/ros/humble/setup.bash

   rosdep install --from-paths src --ignore-src --rosdistro humble -y

   colcon build --symlink-install \
     --packages-select 41068_ignition_bringup

   source install/setup.bash
   ```

   Then use the normal launch commands in step 7. If the team changes the startup
   workflow in a later version, use the instructions for that version.
