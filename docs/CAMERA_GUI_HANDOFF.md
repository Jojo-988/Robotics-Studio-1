# Camera bird map: GUI handoff

## Scope

Display a colour ground-plane mosaic from the drone camera. The mapper publishes
ROS data; rqt is only a test viewer and is not required by the team's GUI.
This is a local planar prototype, not a whole-forest accuracy acceptance.

## Start on a teammate's machine

Build the team package in your own workspace and source that workspace's
install/setup.bash after /opt/ros/humble/setup.bash. Do not copy Zora's absolute
install path. Use ROS_DOMAIN_ID=68 and ROS_LOCALHOST_ONLY=1 in all terminals for
same-machine testing. Cross-machine integration requires a separately agreed
network configuration; localhost-only discovery will not connect two machines.

Reuse one running Parrot simulation with the camera pointing down (pitch pi/2),
and leave it unpaused. Start one mapper:

```bash
ros2 launch 41068_ignition_bringup camera_2d_mapping.launch.py
```

If an existing bridge already publishes /parrot1/camera/camera_info:

```bash
ros2 launch 41068_ignition_bringup camera_2d_mapping.launch.py start_camera_info_bridge:=false
```

These are alternatives, not two commands to run together. The launch starts the
mapper and optionally the CameraInfo bridge. It does not start the simulator or
issue flight commands. Do not restart an already running mapper for this check.

## Minimal GUI implementation

Subscribe with Reliable + Transient Local QoS to:

| Topic | ROS type | Content |
|---|---|---|
| /parrot1/camera_bird_map/image | sensor_msgs/msg/Image | rgba8 mosaic |
| /parrot1/camera_bird_map/valid_mask | sensor_msgs/msg/Image | mono8; 255 sampled, 0 unknown/excluded |
| /parrot1/camera_bird_map/metadata | std_msgs/msg/String | JSON geometry, timestamps, counters |

1. Decode Image using width, height, encoding and step (row stride). Display RGBA
   with alpha over a checkerboard or an explicit unknown-area background.
2. Buffer bounded sets by exact integer (sec, nanosec), pairing both image headers
   with metadata.output_stamp. Display only complete sets. Never join unrelated
   latest messages. Clear pending data when metadata.epoch_id changes, reject
   older versions within an epoch, and clear on simulation-clock rollback.
3. Show valid_area_m2, processed_frames and status. Do not label valid area as
   traversable area or mission completion. Compare source_stamp with simulation
   clock time to detect stale input; no new metadata is guaranteed when input stops.
4. Apply GUI updates on its UI thread. Keep ROS callbacks short and pass completed
   frame sets to that thread. Use the existing team's GUI framework.

Map frame is parrot1_odom. For image column c and row r (not scaled widget pixels):

```text
x = origin_lower_left.x + (c + 0.5) * resolution_m_per_pixel
y = origin_lower_left.y + (height - r - 0.5) * resolution_m_per_pixel
```

Account for widget scaling and letterboxing before converting a click. Row zero
has highest Y; do not vertically flip twice. Alpha is validity, not occupancy.
The image does not need RViz and must not be treated as OccupancyGrid.

## Buttons

Save: /camera_bird_mapper/save_map, std_srvs/srv/Trigger, empty request.
Show returned success/message; the directory belongs to the mapper's machine.
Reset: /camera_bird_mapper/reset_map, same service type. This clears the current
in-memory map; avoid invoking it automatically when the GUI connects.

## Acceptance checklist

- Connect GUI after mapper starts: receive the retained complete map.
- Display colour image and correct unknown area; optionally overlay mask.
- One small verified movement extends coverage without clearing the epoch.
- Save returns success; image, mask and metadata describe the same saved state.
- Pause Gazebo: GUI remains responsive and does not claim fresh frames.
- After saving, deliberate reset clears display and starts a new epoch.

Current live evidence (2026-10-31, user logs): body x=2.265 -> 2.530 m, y=0,
z=0.8 m, unchanged orientation and epoch. Valid pixels 30375 -> 36225; area
3.0375 -> 3.6225 m2. Pairing/TF/invalid/outside counters were zero.
Saved result: 20261031T122036_166308Z_9bb20a49, map_version=1406,
source time=3058.940000000, processed_frames=1405. COMPLETE exists.
This demonstrates local publication, expansion and saving; full seam inspection,
independent geometric accuracy and the team's GUI acceptance remain separate.

Full interface and limitations: CAMERA_2D_MAPPING.md in the same docs directory.
