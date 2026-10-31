# Camera 2D mapping: GUI integration prototype

This component projects colour camera images onto a horizontal ground plane using
the simulation pose at the image timestamp. It is not LiDAR mapping, visual SLAM,
terrain reconstruction, a navigability map, or a validated orthophoto.

## Start

Run a single Parrot simulation separately, with camera pitch pi/2. Keep the same
ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY settings in every process. The mapper launch
does not start Gazebo/RViz and never publishes flight commands.

After building and sourcing the package:

```bash
ros2 launch 41068_ignition_bringup camera_2d_mapping.launch.py
```

This starts the mapper and the Fortress CameraInfo bridge. If CameraInfo already
has a bridge, use `start_camera_info_bridge:=false`. Stop the temporary manual
bridge before using the default launch. Do not run duplicate mapper instances.
Optional launch arguments: `params_file`, `use_sim_time`.

Default canvas: x=[-2,6), y=[-4,4), 800x800, 0.01 m/pixel. Processing is limited
to 1 Hz wall time. This is a local integration canvas, not the whole forest.
For a 32x32 m forest canvas, a practical starting configuration is origin
(-16,-16), resolution 0.05, width=height=640. Parameters are read-only after
startup: edit the YAML and restart, which starts an empty mosaic.

## GUI contract

All outputs use Reliable, Transient Local, Keep Last depth 1:

| Topic | Type / encoding |
|---|---|
| `/parrot1/camera_bird_map/image` | sensor_msgs/Image, rgba8 |
| `/parrot1/camera_bird_map/valid_mask` | sensor_msgs/Image, mono8 |
| `/parrot1/camera_bird_map/metadata` | std_msgs/String containing JSON |

The image is a georeferenced raster, not a raw perspective camera image. Frame
is `parrot1_odom`; do not feed it to camera calibration/projection tools as a
raw image. Right is +X, up is +Y. For column c and PNG/ROS image row r:

```
x = origin_x + (c + 0.5) * resolution
y = origin_y + (height - r - 0.5) * resolution
```

Alpha/mask 255 means a valid planar-projection sample; 0 means unobserved or
excluded. It does not indicate free/occupied ground. Do not use black RGB as
an invalidity test. Latest valid pixels replace older pixels; missing pixels
never erase existing coverage. No image registration or feature stitching is
applied. Repeated frames, shadows and raised objects can still change colours.

Pair image.header.stamp and mask.header.stamp with JSON `output_stamp`.
`source_stamp` is the last accepted source image (or reset clock time for an
empty map), not always equal to output_stamp. `epoch_id` changes on reset or
clock rollback; flush GUI pairing caches then. `map_version` is monotonic only
within an epoch. Buffer bounded groups by output_stamp, display only complete
groups and discard old groups; do not combine three unrelated latest messages.
After a simulator restart, also discard old source-clock data.

Metadata includes origin, dimensions, resolution, processed_frames, valid_pixels,
valid_area_m2, last_outside_pixels and drop counters. Valid area is not a forest
mission completion percentage. Inspect `source_stamp` age for stale data: there
is no heartbeat when source images stop. `superseded_pairs` is intentional rate
limiting; mismatched/expired images and invalid projections are separate counts.
The transport is non-atomic: a subscriber may briefly receive only part of a group.

## Reset and save

```bash
ros2 service call /camera_bird_mapper/reset_map std_srvs/srv/Trigger '{}'
ros2 service call /camera_bird_mapper/save_map std_srvs/srv/Trigger '{}'
```

Reset clears mosaic and pairing buffers, publishes an empty version, and rejects
images at or before reset time. Save returns a unique UTC-named result directory
under `~/team-results/drone-camera/live` by default, with bird_view.png,
valid_mask.png, metadata.json and COMPLETE. A directory without COMPLETE is
incomplete. Saving is synchronous; large maps can briefly delay callbacks.

## Geometry and limitations

This prototype explicitly assumes the Gazebo RGBD sensor is mounted at
parrot1_camera_link with no additional sensor pose. Optical xyz -> link xyz is
(z,-x,-y), at the same optical centre. It does not use the offset
camera_rgb_optical_frame link. Changing the robot model requires revalidation.
Only zero-distortion, unbinned RGB/BGR/RGBA/BGRA inputs are supported. Exact
Image/CameraInfo timestamps and frame IDs must agree; TF is queried at that
timestamp, never at latest time. Rays crossing the horizon are rejected.

Ground is assumed z=0. Bottom 10% of source rows are manually excluded for the
current sample's white-object region. This is not automatic obstacle masking.
Elevated objects, terrain and changing illumination can cause misalignment.
Canvas clipping is counted; there is no automatic canvas growth or movement.

## Implementation and evidence

- map_data/camera_projection.py: shared pure projection calculation.
- map_data/camera_mosaic.py: bounded mosaic, clipping, valid-pixel updates.
- scripts/project_camera_sample.py: offline CLI using the same projector.
- scripts/camera_bird_mapper.py: exact-time ROS input/output, reset/save.
- config/camera_2d_mapping.yaml and launch/camera_2d_mapping.launch.py.

The saved A/B experiment ab-test-20261031-222802 was compared against the shared
projector: both outputs had zero RGBA differences, zero mask differences and
identical metadata. Four unit tests passed using unittest. An isolated ROS-domain
test fed the saved A/B images and synthetic timestamps/transforms, and verified
publication, mismatch rejection, save pixels, reset, old-frame rejection, clock
rollback and recovery. These checks do not validate absolute map accuracy or
constitute live Gazebo/GUI acceptance. The 0.265 m real-simulation A/B experiment
predates this refactor and supports limited fixed-height translation only.

Run pure tests from the repository root:

```bash
python3 -m unittest discover -s test -p test_camera_projection.py -v
```

Build currently emits the repository's package-naming warning and a missing
pytest warning. The direct unittest/isolated ROS checks above were run; no claim
is made that all colcon/pytest tests ran. No Git commit or push is performed by
this integration task.
