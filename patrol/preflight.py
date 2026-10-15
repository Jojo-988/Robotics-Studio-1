"""Startup checks shared by the shell helper and the ROS launch entry point."""
import argparse
import importlib.util
import os
from pathlib import Path
import shutil
import sys


PACKAGE = '41068_ignition_bringup'


def check_environment(mode='standalone', workspace=None):
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    errors, notes = [], []
    dependencies = ['rclpy', 'launch', 'launch_ros', 'ros2launch', 'geometry_msgs', 'nav_msgs',
                    'sensor_msgs', 'rosgraph_msgs', 'std_msgs', 'std_srvs']
    if mode == 'standalone':
        dependencies += ['ros_ign_gazebo', 'ros_ign_bridge', 'robot_state_publisher',
                         'robot_localization', 'xacro']
    for name in [PACKAGE] + dependencies:
        try:
            get_package_prefix(name)
        except LookupError:
            errors.append('Missing ROS package: ' + name)
    for name in ('numpy', 'rclpy', 'yaml', 'patrol.health'):
        try:
            found = importlib.util.find_spec(name)
        except (ImportError, ValueError):
            found = None
        if found is None:
            errors.append('Missing Python module: ' + name)
    if os.environ.get('ROS_DISTRO') != 'humble':
        errors.append('Source /opt/ros/humble/setup.bash; this project is validated on Humble/Fortress.')
    try:
        domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
        if not 0 <= domain <= 232:
            raise ValueError
        if domain > 101:
            notes.append('ROS_DOMAIN_ID > 101 may overlap Linux ephemeral ports; prefer 0..101.')
    except ValueError:
        errors.append('ROS_DOMAIN_ID must be an integer between 0 and 232.')
    if os.environ.get('ROS_LOCALHOST_ONLY', '0') not in ('0', '1'):
        errors.append('ROS_LOCALHOST_ONLY must be 0 or 1.')
    try:
        prefix = Path(get_package_prefix(PACKAGE)).resolve()
        share = Path(get_package_share_directory(PACKAGE))
        notes.append('Active package: ' + str(prefix))
        if workspace and not prefix.is_relative_to((Path(workspace) / 'install').resolve()):
            errors.append('Another workspace shadows this package. Rebuild and source the intended workspace.')
        for relative in ('config/coverage_patrol.yaml', 'launch/41068_coverage_patrol.launch.py'):
            if not (share / relative).is_file():
                errors.append('Installed file missing: ' + relative)
        executable = prefix / 'lib' / PACKAGE / 'coverage_patrol.py'
        if not os.access(executable, os.X_OK):
            errors.append('coverage_patrol.py is missing or not executable; rebuild the package.')
        if mode == 'standalone' and not shutil.which('ign'):
            errors.append('Ignition CLI not found; install Gazebo Fortress / ros-humble-ros-ign-gazebo.')
    except LookupError:
        pass
    if mode == 'standalone' and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        notes.append('No display detected. GPU camera/lidar rendering also needs a working rendering context; '
                     'gz_gui:=false alone does not provide one.')
    notes += [f'Mode: {mode}; ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "0")}; '
              f'ROS_LOCALHOST_ONLY={os.environ.get("ROS_LOCALHOST_ONLY", "0")}',
              'Runtime readiness will separately check clock, odometry, controller ownership and sensors.']
    return errors, notes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('standalone', 'attach'), default='standalone')
    parser.add_argument('--workspace')
    args = parser.parse_args()
    errors, notes = check_environment(args.mode, args.workspace)
    for note in notes:
        print('[preflight] ' + note)
    for error in errors:
        print('[preflight ERROR] ' + error, file=sys.stderr)
    if errors:
        print('From the workspace root: rosdep install --from-paths src --ignore-src -r -y; '
              'then colcon build --symlink-install and source install/setup.bash', file=sys.stderr)
    return bool(errors)


if __name__ == '__main__':
    raise SystemExit(main())
