#!/usr/bin/env bash
# Source checkout helper. The ROS launch file owns simulation/partition/rendering policy.
set -eo pipefail

patrol_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
patrol_mode=standalone
patrol_legacy=auto
patrol_check_only=false
patrol_args=()
for patrol_arg in "$@"; do
  case "$patrol_arg" in
    --check) patrol_check_only=true ;;
    mode:=standalone|mode:=attach) patrol_mode="${patrol_arg#mode:=}"; patrol_args+=("$patrol_arg") ;;
    mode:=*) printf '%s\n' 'mode must be standalone or attach' >&2; exit 2 ;;
    start_sim:=false) patrol_legacy=attach; patrol_args+=("$patrol_arg") ;;
    start_sim:=true) patrol_legacy=standalone; patrol_args+=("$patrol_arg") ;;
    start_sim:=auto) patrol_legacy=auto; patrol_args+=("$patrol_arg") ;;
    start_sim:=*) printf '%s\n' 'start_sim must be auto, true or false' >&2; exit 2 ;;
    *) patrol_args+=("$patrol_arg") ;;
  esac
done
[[ "$patrol_legacy" == auto ]] || patrol_mode="$patrol_legacy"
# Walk upwards: checkout names and src nesting may differ between teammates.
patrol_workspace="${PATROL_WORKSPACE:-}"
if [[ -z "$patrol_workspace" ]]; then
  patrol_candidate="$patrol_script_dir"
  while [[ "$patrol_candidate" != / ]]; do
    if [[ -f "$patrol_candidate/install/setup.bash" ]]; then
      patrol_workspace="$patrol_candidate"
      break
    fi
    patrol_candidate="$(dirname -- "$patrol_candidate")"
  done
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  printf '%s\n' 'ROS 2 Humble not found: /opt/ros/humble/setup.bash' >&2
  exit 1
fi
if [[ -z "$patrol_workspace" || ! -f "$patrol_workspace/install/setup.bash" ]]; then
  printf '%s\n' 'Built workspace not found. Run colcon build, or set PATROL_WORKSPACE to its root.' >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
source "$patrol_workspace/install/setup.bash"

if [[ "$patrol_mode" == standalone ]]; then
  export ROS_DOMAIN_ID="${PATROL_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-68}}"
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
else
  # Match the existing simulation: do not silently move it to an isolated domain.
  export ROS_DOMAIN_ID="${PATROL_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-0}}"
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
fi
printf 'Patrol workspace: %s\n' "$patrol_workspace"
python3 -m patrol.preflight --mode "$patrol_mode" --workspace "$patrol_workspace"
if [[ "$patrol_check_only" == true ]]; then
  exit 0
fi
exec ros2 launch 41068_ignition_bringup 41068_coverage_patrol.launch.py "${patrol_args[@]}"
