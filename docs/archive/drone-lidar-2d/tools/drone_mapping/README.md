> **Archived 2026-11-01 — retired drone LiDAR 2D prototype.** Commands and results below are historical; removed runtime code is available at Git commit `e1e091e`. Use the current camera mapping documentation for GUI integration.

# Offline drone mapping validation tools

These tools read saved data. They do not publish ROS commands, move the drone,
reset a map or control Gazebo. Run them from the repository root in WSL.
They are source-tree utilities, not installed ROS executables.

## Tests

Python 3 and NumPy are required for geometry tests. The wall tool also imports
PyYAML and Pillow. The bag reader additionally requires the ROS 2 Humble Python
environment (`rclpy`, `sensor_msgs_py`, `tf2_ros`, `rosidl_runtime_py`).

```bash
source /opt/ros/humble/setup.bash
python3 -m unittest discover -s tools/drone_mapping -p 'test_*.py' -v
```

## Wall scale check

```bash
python3 tools/drone_mapping/check_map_scale.py --help
```

Supply `--map` with a full OccupancyGrid YAML capture, `--world` with
`worlds/large_demo.sdf`, and `--output` with a new result directory outside the
repository. The tool checks the inspected large_demo wall layout. Its ROIs assume
approximately aligned map/world axes; it is not a generic registration algorithm.
It creates a report and an overlay, with status MEASURED_NOT_ACCEPTANCE.

## Three-dimensional landmark candidates

```bash
python3 tools/drone_mapping/fit_bag_landmarks.py --help
```

Supply `--bag` with the SQLite `.db3` file and `--output` with a new directory.
This version supports one SQLite bag file, not split bags or MCAP. It expects
`/parrot1/scan/points`, `/parrot1/tf` and `/parrot1/tf_static`.

The inspected scene profile is explicit in the source: tree at (4,-4), animal
at (-4,4), target frame parrot1_odom. Do not use this profile for another scene
without reviewing the geometry and ROIs. The radius/centre fit is free, but
candidate selection uses the nominal scene. Geometry gates (12 points, 60 degree
XY arc, condition number <=1000, p95 residual <=0.05 m) are diagnostic choices,
not project acceptance requirements.

Interpret output alongside the scene-state evidence. This tool does not read
Gazebo truth files, so its generic historical-truth limitation remains present
even if separately recorded truth samples exist. It must not be used to claim
whole-map accuracy or complete forest coverage.

For curated results and exclusions, see
[validation index](../../docs/DRONE_2D_MAPPING_VALIDATION.md).
