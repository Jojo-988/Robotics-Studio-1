#!/usr/bin/env bash
# Run from a source checkout at <workspace>/src/<repository>/scripts/.
set -e

patrol_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
patrol_workspace="${PATROL_WORKSPACE:-$(cd -- "$patrol_script_dir/../../.." && pwd)}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  printf '%s\n' 'ROS 2 Humble was not found at /opt/ros/humble.' >&2
  exit 1
fi
if [[ ! -f "$patrol_workspace/install/setup.bash" ]]; then
  printf 'Workspace setup not found: %s/install/setup.bash\n' "$patrol_workspace" >&2
  printf '%s\n' 'Build this workspace first, or set PATROL_WORKSPACE to its root directory.' >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source "$patrol_workspace/install/setup.bash"

export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${PATROL_ROS_DOMAIN_ID:-192}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
# ROS domains alone do not isolate Gazebo Transport. Use a fresh partition
# for each launch so a leftover simulator cannot supply another clock/odometry.
export IGN_PARTITION="patrol_view_$(date +%s)_$$"
export GZ_PARTITION="$IGN_PARTITION"

printf 'Patrol workspace: %s\nROS_DOMAIN_ID: %s\nGazebo partition: %s\n' \
  "$patrol_workspace" "$ROS_DOMAIN_ID" "$IGN_PARTITION"

exec ros2 launch 41068_ignition_bringup 41068_coverage_patrol.launch.py "$@"
