# Drone 2D Bird's-eye Mapping

## Status and scope

As of 2026-10-31, this is a working LiDAR grid-classification prototype
using simulation-assisted localisation. It publishes a coordinate-referenced
2D map, statistics and a reset service. Simple-scene projection, accumulation,
short movement and saved-map reload have been tested. Forest quality validation
is ongoing: sparse, changing returns can become persistent structure cells.

The output is not a colour aerial mosaic, calibrated occupancy probability
map, independently localised SLAM result or validated navigation map.
The team/instructor should confirm that this grid representation meets the
bird's-eye mapping deliverable, including its required area and accuracy.

## Processing and localisation

1. Receive `/parrot1/scan/points` (`sensor_msgs/msg/PointCloud2`).
2. Extract XYZ and discard non-finite coordinates.
3. Transform from the cloud header frame (observed: `parrot1_base_scan`)
   into `parrot1_odom` at the cloud acquisition timestamp.
4. Classify point heights, project them into a fixed XY grid and count
   at most one hit per cell, per category, per accepted frame.
5. Publish the single-frame preview, accumulated classifications and statistics.

TF uses `/parrot1/tf` and `/parrot1/tf_static`. The launch file supplies these
remappings. Simulation time comes from `/clock`.
The current localisation configuration fuses the full pose from simulation
odometry `/parrot1/odometry`; `/parrot1/odom` and body TF are its outputs.
Agreement between these related sources is not independent pose accuracy evidence.

## Configuration and current cell rules

| Setting | Simple-scene default | Forest experiment |
|---|---|---|
| Configuration | `config/drone_2d_mapping.yaml` | `config/drone_2d_mapping_forest.yaml` |
| Resolution | 0.1 m/cell | 0.1 m/cell |
| Width x height | 200 x 200 | 320 x 320 |
| Origin (x, y) | (-10, -10) m | (-16, -16) m |
| Extent on each axis | [-10, 10) m | [-16, 16) m |
| Ground reference | z = 0 m | z = 0 m |
| Ground candidate | abs(z - ground_z) <= 0.05 m | Same |
| Structure candidate | z - ground_z >= 0.15 m | Same |
| Confirmation threshold | 3 accumulated frames | 3 accumulated frames |

| Published value | Current accumulated-map meaning |
|---|---|
| -1 | Insufficient or unresolved evidence |
| 0 | At least 3 ground-hit frames and no structure-hit frame |
| 100 | At least 3 structure-hit frames; structure takes priority |

The current implementation counts hits across the whole session, not a rolling
window. Confirmed structures persist until reset or restart. A tentative
structure hit blocks a ground classification even before structure confirmation.
Missing returns do not prove free space. Ground candidates do not certify
traversability. A mixed-evidence cell is included in the structure total,
not an additional fourth map class.

The proposed 3-of-5-frame structure rule has only been compared offline;
it is not implemented in the running mapper. See the experiment log.

## ROS interfaces

Node: `/drone_cloud_diagnostic`.

| Topic | Type | Purpose |
|---|---|---|
| `/parrot1/bird_map` | `nav_msgs/msg/OccupancyGrid` | Accumulated map for consumers |
| `/parrot1/bird_map_statistics` | `std_msgs/msg/String` | JSON v1 statistics from the same map snapshot |
| `/parrot1/bird_map_preview` | `nav_msgs/msg/OccupancyGrid` | Single-frame diagnostic preview |
| `/parrot1/bird_map_accumulated_preview` | `nav_msgs/msg/OccupancyGrid` | Accumulated diagnostic output |

Map and statistics publishers use Reliable, Transient Local, Keep Last, depth 1.
The cloud subscriber uses sensor-data QoS (Best Effort, Volatile).
`map_topic` and `statistics_topic` configure the two consumer outputs.
Preview topics are currently fixed names. Consumers must check source timestamps:
a retained message does not prove that mapping is still updating.

### Statistics JSON v1

| Field | Meaning |
|---|---|
| `schema_version` | 1 |
| `map_kind` | `lidar_bird_eye_classification` |
| `frame_id` | Target mapping frame |
| `source_stamp` | Source cloud stamp (`sec`, `nanosec`); reset uses reset time |
| `width`, `height`, `resolution` | Grid dimensions and metres per cell |
| `origin` | Object with `x`, `y`, `yaw`; current yaw is 0 |
| `accumulated_frames` | Accepted frames since session start/reset |
| `mixed_cells` | Confirmed structure cells also having sufficient ground support |
| `tf_drops`, `busy_skips` | Node processing counters; not a complete transport-loss count |
| `total_cells` | Width multiplied by height |
| `ground_candidate_cells` | Number of cells with value 0 |
| `structure_cells` | Number of cells with value 100 |
| `unknown_cells` | Number of cells with value -1 |
| `classified_fraction` | (Ground candidate + structure cells) / total cells |

This schema differs from the Husky map-statistics interface. In particular,
`classified_fraction` is not forest survey completion, and ground candidates
are not a substitute for certified free-space cells.

### Reset service

`/drone_cloud_diagnostic/reset_map` uses `std_srvs/srv/Trigger`.
It clears counts, pending work and session counters, publishes an all-unknown
map and zero statistics, and accepts clouds captured after the reset time.
New observations repopulate the map. Reset discards the in-memory session;
save any desired snapshot first. Restart the mapper if simulation time resets.

## Run against an existing simulation

Build and source the workspace containing this package. Start the drone
simulation separately, with the same ROS domain and local-host settings.
The mapping launch file does not start Gazebo or RViz.

```bash
ros2 launch 41068_ignition_bringup drone_2D_mapping.launch.py
```

The filename contains an uppercase `D`. To use the installed forest configuration:

```bash
ros2 launch 41068_ignition_bringup drone_2D_mapping.launch.py \
  params_file:="$(ros2 pkg prefix 41068_ignition_bringup)/share/41068_ignition_bringup/config/drone_2d_mapping_forest.yaml"
```

In RViz, use Fixed Frame `parrot1_odom`, Map topic `/parrot1/bird_map`,
Reliable / Transient Local QoS and a top-down view. Match the simulation TF
remappings if displaying live sensor data.

## Saving, verification and remaining acceptance work

Keep the image and YAML together. The verified trinary PGM sample uses
`negate: 0`, `free_thresh: 0.15` and `occupied_thresh: 0.65` to preserve its
unknown pixels. These settings describe this encoding, not all image formats.
Compare frame, geometry and every cell after independent reload; timestamps
are expected to differ. Image/YAML export does not save observation counters
and cannot resume an accumulator session.

- [Experiment log](DRONE_2D_MAPPING_TEST_LOG.md)
- [Verified saved-map sample](results/drone-bird-map/20261031-122803/README.MD)

Remaining work: validate the candidate filtering rule during motion; check
landmark positions and scale; capture a declared forest area with a recorded
route; repeat export/reload checks for the final map. Ground-height assumptions,
occlusion, fixed extent, persistent false structures and simulation-assisted
pose remain limitations. No whole-forest accuracy or coverage acceptance is claimed.
