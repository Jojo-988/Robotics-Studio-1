# Drone 2D Mapping Experiment Log

## 2026-10-31: Initial projection and accumulation tests

Sections 1-10 record the initial prototype session. Later results and the
current status are recorded in the follow-up sections below.

### 1. Experiment Objectives

Transform drone LiDAR point clouds into a fixed coordinate frame to generate
a 2D grid preview, and verify whether historical observations are retained
after a short movement.

### 2. Experimental Environment

- Operating system: WSL Ubuntu 22.04
- ROS version: ROS 2 Humble
- Simulation environment: Gazebo / Ignition
- Scene: `simple_trees`
- Robot: `parrot1`
- Diagnostic program: `scripts/drone_cloud_diagnostic.py`
- Point cloud input: `/parrot1/scan/points`
- Point cloud frame: `parrot1_base_scan`
- Fixed frame: `parrot1_odom`
- Single-frame preview: `/parrot1/bird_map_preview`
- Accumulated preview: `/parrot1/bird_map_accumulated_preview`

### 3. Pose Source

The current prototype uses simulation odometry to assist mapping.

`robot_localization` fuses the full position and orientation from
`/parrot1/odometry` and publishes `/parrot1/odom` and the robot body TF.

Runtime parameters:

- `two_d_mode`: `false`
- `odom0_config`: the first six entries are `true`; all remaining entries are `false`
- `odom0_differential`: `false`
- `odom0_relative`: `false`

This configuration is used to validate the mapping pipeline. It does not
demonstrate an independent autonomous SLAM system.

### 4. Map and Classification Configuration

| Item | Configuration |
|---|---|
| Map frame | `parrot1_odom` |
| Resolution | 0.1 m/cell |
| Width | 200 cells |
| Height | 200 cells |
| Total cells | 40000 |
| Origin | (-10, -10) m |
| Horizontal extent | x and y both within [-10, 10) m |
| Ground reference | z = 0 m |
| Ground candidate | `abs(z) <= 0.05 m` |
| Structure candidate | `z >= 0.15 m` |
| Other valid points | Unclassified |
| Accumulation support threshold | 3 frames |

Accumulation rules:

- Each cell can receive at most one ground-count increment and one
  structure-count increment per frame.
- A cell is displayed as a structure candidate after structure candidate
  returns have been observed in at least 3 frames.
- A cell is displayed as a ground candidate after ground candidate returns
  have been observed in at least 3 frames, provided that no structure
  candidate returns have been recorded in that cell.
- All other cells remain unknown.
- Historical ground and structure counts are retained separately.
- Missing returns are not treated as free space and do not clear historical
  structure evidence.

`OccupancyGrid` is currently used as a diagnostic display format:

- `-1`: Unknown or insufficient evidence
- `0`: Ground candidate
- `100`: Structure candidate

These values do not represent calibrated occupancy probabilities and are
not used directly for navigation.

### 5. Implemented Functions

- Receive `PointCloud2` messages and extract XYZ coordinates.
- Filter invalid coordinates, including NaN and Inf.
- Look up the TF corresponding to the point cloud acquisition timestamp.
- Transform point clouds into `parrot1_odom`.
- Report height distributions and statistics within the fixed region.
- Classify ground and structure candidates using height rules.
- Project points onto a fixed 2D grid.
- Publish a single-frame 2D preview.
- Maintain observation counts across frames and publish an accumulated preview.
- Compare the point cloud, single-frame grid and accumulated grid in RViz.

### 6. Validation Procedure and Results

#### 6.1 Stationary Pose Check

After updating the localisation configuration:

- The raw position was approximately (2.000, 0.000, 0.800) m.
- The filtered position was approximately (2.000, 0.000, 0.800) m.
- The robot body TF agreed with the filtered position.

Result: The pose pipeline was consistent under stationary conditions.

#### 6.2 Altitude Increase Test

- The initial altitude was approximately 0.800 m.
- After ascending, the raw altitude was approximately 1.396 m.
- The filtered altitude was approximately 1.396 m.
- The robot body TF consistently reported an altitude of approximately 1.396 m.

Result: The filtered pose and TF followed the altitude change reported by
the simulation odometry.

#### 6.3 Point Cloud and Height Statistics

During one stationary test at an altitude of approximately 1.396 m:

- Each frame contained 23040 points.
- There were 21203 valid XYZ points, approximately 92.0% of the total.
- The median height within the fixed region of interest (ROI) was
  approximately 0 m.
- The 5th-to-95th percentile height range within the ROI was approximately
  -0.016 to 0.015 m.

Result: Candidate ground heights were concentrated near zero, and the
distribution remained broadly stable while stationary.

This finding applies only to the current scene and inspected region.

#### 6.4 Single-Frame 2D Projection

- Ground candidate, structure candidate and unknown cells summed to 40000.
- The point cloud and 2D preview were broadly aligned in the screenshots.
- All valid points in the current test were within the grid boundaries.
- Unobserved regions remained unknown. No hole filling or ray-based
  clearing was applied.

Result: The single-frame projection and display passed the basic checks.

#### 6.5 Stationary Multi-Frame Accumulation

At frame 558 of one stationary test, the accumulated results were:

- Ground candidate cells: 4221
- Structure candidate cells: 68
- Mixed-evidence cells: 28
- Unknown cells: 35711

The counts satisfied:

```text
4221 + 68 + 35711 = 40000
```

Mixed-evidence cells were a subset of the structure candidate cells and
were not counted separately in the total.

Result: Multi-frame counting and accumulated-map publication operated
as expected.

An increase in classified cells while stationary does not indicate
exploration of new areas. It may also result from repeated observations,
noise-induced variation and cells reaching the support threshold.

#### 6.6 Short Horizontal Movement

This test was conducted at an altitude of approximately 0.8 m after
restarting the simulation.

- The initial position was approximately (2.000, 0.000, 0.800) m.
- The raw position after movement was approximately (2.301, 0.000, 0.800) m.
- The displacement along the x-axis was approximately 0.301 m.
- The filtered position remained broadly consistent with the raw position
  after movement.
- The accumulated grid contained more observations than the current
  single-frame grid.
- No obvious structure splitting or overall misalignment was visible
  in the screenshots.

Result: The accumulated display passed the basic check following a short
movement. This test does not replace longer-distance motion testing or
map accuracy validation.

### 7. Limitations at the initial test stage

- Mapping relies on simulation pose data; localisation with real sensors
  has not been validated.
- The raw and filtered poses are derived from related data sources.
  Their agreement does not establish independent localisation accuracy.
- The height classification rules are designed for the current flat-ground
  scene and cannot be directly generalised to sloping terrain.
- Ground candidate cells do not guarantee traversability.
- The identities and dimensions of structures represented by structure
  candidate cells have not been individually verified.
- The three-frame support rule cannot eliminate persistent false returns.
- Dynamic-object handling, evidence decay and comprehensive conflict
  handling have not been implemented.
- The map extent is fixed; automatic expansion has not been implemented.
- Accumulated data are stored only in memory and are cleared when the
  node restarts.
- Longer-distance motion, turning and multi-view consistency have not
  yet been validated.
- Map saving, independent reloading and class preservation have not
  yet been verified.

### 8. Interim Conclusion

A prototype for drone point-cloud projection onto a 2D grid and multi-frame
accumulation of candidate evidence has been implemented using simulation
pose data.

The stationary, altitude-change and short horizontal-movement tests support
the conclusion that the current processing pipeline is functioning at
a basic level.

The current output is a local 2D map preview. It cannot yet be regarded
as a complete, accuracy-validated bird's-eye map of the forest.

### 9. Next steps recorded at the initial test stage

1. Save the accumulated map image and YAML metadata.
2. Independently reload the saved map.
3. Compare the dimensions, resolution, origin and counts of all three
   cell classes before and after saving.
4. Specifically verify that unknown cells are preserved.
5. Extend the movement tests to include turning and observations from
   multiple viewpoints.
6. Validate structure positions and map scale against scene landmarks.

### 10. Experiment Screenshots

To be archived:

- Point cloud overlaid with the single-frame 2D preview.
- Accumulated map while stationary.
- Accumulated map after horizontal movement.
- Single-frame map at the same position.

## 2026-10-31 follow-up: consumer interfaces and reset

The mapper now publishes `/parrot1/bird_map` and JSON statistics on
`/parrot1/bird_map_statistics`, both Reliable / Transient Local, depth 1.
The two diagnostic preview topics remain available. The exact interface and
cell rules are described in [DRONE_2D_MAPPING.md](DRONE_2D_MAPPING.md).

The reset service `/drone_cloud_diagnostic/reset_map` returned success.
The observed reset snapshot had 0 accumulated frames, 0 ground cells,
0 structure cells and 40000 unknown cells. A later snapshot had 41 frames,
3750 ground candidates, 58 structures and 36192 unknown cells, demonstrating
that accumulation resumed. Both snapshots reported zero TF drops and busy skips.
These counters do not exclude losses elsewhere in the transport pipeline.

## 2026-10-31 follow-up: saved-map class preservation

The [12:28:03 sample](results/drone-bird-map/20261031-122803/README.MD)
contains the original and reloaded ROS grid messages plus the PGM/YAML pair.
Both messages contain 33578 unknown, 6341 ground-candidate and 81 structure
cells. The frame and geometry agree and the cell-by-cell difference is zero.
The checked counts total 40000 cells.

An earlier reload converted unknown pixels to free cells. The retained reload
configuration uses trinary mode, negate 0, free threshold 0.15 and occupied
threshold 0.65. The corrected server was cleaned up, configured and activated
again before capturing the reloaded message. Reload success does not establish
landmark accuracy or navigability.

## 2026-10-31 follow-up: bounded multi-position acquisition

A separate `simple_trees` trial (`run-20261031-171755`) followed the nominal
route (2,0) -> (2.6,0) -> (2.6,-0.6) -> (2,-0.6) -> (2,0), at z=0.8 m.
The raw simulation odometry return error was 0.011873 m. This is a vehicle
return measurement, not a map-position error or loop-closure accuracy result.

The final saved snapshot contained 205 accumulated frames, 11462 ground
candidates, 85 structures and 28453 unknown cells. Its export/reload comparison
reported matching frame and geometry with zero changed cells. TF drops and
busy skips at the recorded stops were zero.

The detailed route snapshots and report remain in the local Windows project
under `mapping-test-20261031/run-20261031-171755`; they are not included in the
12:28:03 sample directory and should not be confused with that earlier run.
This validates bounded multi-position accumulation and class-preserving export,
not long-range map accuracy or complete forest coverage.

## 2026-10-31 follow-up: forest stationary diagnostic

Scene: `large_demo`. Grid: 320 x 320, resolution 0.1 m, origin (-16,-16) m.
Dense southwest structure streaks appeared in the accumulated map while
single-frame previews showed much sparser returns. The structure total grew
while stationary. This observation requires investigation rather than being
counted as new surveyed area.

The first diagnostic bag contained no point clouds or dynamic pose/TF messages
and was excluded. The usable bag was
`diagnostic-20261031-174614/stationary_cloud` (local experiment storage):
53 clouds, 349 raw odometry messages, 553 filtered odometry messages,
553 dynamic TF messages, one static TF message and 39 statistics messages,
over about 19.26 seconds of recording time.

Read-only analysis of this bag found:

- Raw position stayed at (2,0,0.8) m; filtered XYZ also showed no variation.
- Maximum filtered orientation change was approximately 0.000943 degrees.
- Transforming with time-matched TF versus the first fixed pose changed points
  by at most approximately 0.000132 m and produced identical confirmed structure
  masks under fresh accumulation (774 cells).
- Consecutive returns at matching beam indices sometimes differed by more than
  1 m; all such jumps in this recording involved the inspected southwest region.

These findings do not support substantial TF drift as the explanation in this
bag. Changing raw returns combined with unlimited cumulative confirmation are
a plausible contributor. The physical/simulation cause of those changing
returns, and which cells are genuinely false, have not been established.

## 2026-10-31 follow-up: offline candidate rule comparison

The independent offline tool compared fresh accumulation on the same 53 clouds:

- Baseline: confirm a structure after 3 hits anywhere in the session.
- Candidate: once a full 5-frame window is available, confirm a structure if
  at least 3 of those frames hit the cell; retain confirmed structures thereafter.

Each frame contributes at most one hit per cell. All 53 clouds were transformed
successfully. The five-frame windows spanned about 1.33-1.34 simulation seconds.
Checks covered sparse-hit rejection, confirmation after a full window and
retention after subsequent misses.

| Region in parrot1_odom | Baseline | Candidate | Retained relative to baseline |
|---|---:|---:|---:|
| Whole grid | 774 | 569 | 73.5% |
| Southwest: x [-13,-4), y [-13,-2) m | 232 | 66 | 28.4% |
| Top boundary: x [-12,12), y [12,13) m | 123 | 114 | 92.7% |
| Right boundary: x [12,13), y [-10,10) m | 128 | 120 | 93.8% |

These are baseline-relative retention measurements, not accuracy against ground
truth. Sparse true structures can also be removed. The comparison started from
empty counts and cannot be compared directly with the longer live session's
structure total. It does not validate moving-flight performance or guarantee
that long-running false accumulation will stop.

At the time of this offline comparison, the candidate rule had not yet been
integrated. Subsequent window-mode integration is recorded below. Offline outputs and the tool
remain in the local Windows project under `mapping-test-20261031`; the tool's
bag path needs parameterisation before publishing it as a reusable team utility.

## Status recorded before window integration (historical)

- Grid unit tests: 6 passed; existing map-statistics tests: 7 passed on review.
- Working prototype: timestamped point transformation, classification, cumulative
  mapping, consumer topics, reset and sample export/reload.
- Verified samples do not certify forest geometry, navigability or full coverage.
- The fixed-height classifier is designed for the inspected flat-ground scene.
  Low-altitude returns do not establish a complete canopy map.
- Next: introduce a selectable window rule with reset tests, compare the formal
  node against the offline result, validate during movement, then perform declared
  forest-area acquisition and landmark/scale checks before final export.

Local reports referenced above are supporting experiment records, not files
already included in this repository. The committed sample file list is in its
own README; missing screenshots must not be presented as archived evidence.


## 2026-10-31: window integration and bounded forest motion

Window mode (5 frames, 3 hits) is now integrated. The production grid module
matched all 53 offline reference frames, with zero differing structure cells;
final baseline/candidate counts were 774/569. Reset cleared the window fill to
zero and all 102400 cells to unknown; resumed acquisition rebuilt the window.

A bounded forest trial preserved the existing map and moved from (2,0,0.8)
to (2.288629,0,0.8), returning to (2.010895,0,0.8) m. Return error was 0.010895 m
in simulation odometry, not an independent map accuracy measurement.
Final snapshot: 547 frames, 7300 ground candidates, 1116 structures, 93984 unknown.
No old confirmed structures disappeared. Of 454 new structure cells, 392 lay
within one cell of old structures; this alone cannot distinguish newly observed
surfaces from thickening. TF drops at recorded stops were zero.
Independent save/reload preserved frame, geometry and all cells (zero differences).

The A snapshot had mismatched map/statistics timestamps. Its image-array checks
remain usable, but it must not be treated as a time-paired snapshot. B, return
and final snapshots matched. The verification helper now pairs by integer source
timestamp and checks counts before accepting output. Regression testing rejects
the old A pair and accepts the other three. Local detailed evidence is in
`mapping-test-20261031/window-motion-20261031-193607` in the Windows project.

Remaining acceptance work: declared-area acquisition, landmark/scale evaluation,
turning and longer-route validation. No full-forest or navigation certification.


## 2026-10-31: scale and local landmarks — latest follow-up

Selected left/right wall spacing was approximately 25.0 m against the nominal
25 m. This supports scale consistency in those regions, not full-map accuracy.
The scene file used by the running simulator matched the source-file hash.

A stationary follow-up captured 64 clouds with periodic Gazebo entity states.
The inspected entities and raw odometry showed no pose change across recorded
samples. Historical TF lookup succeeded for all 64 clouds. Tree fitting supported
58 frames: median XY deviation 0.0206 m, median fitted radius 0.1728 m against
0.18 m. The held-out right lower wall segment had 1600 candidate returns with
p95 absolute surface deviation 0.0409 m. These are local raw-cloud measurements;
the accumulated grid is coarser and includes earlier observations.

Animal fitting remained unsupported (3–4 points/frame). The earlier bottom-wall
check also had no observations and was not counted as passed. Mapper accepted-frame
count increased by 33 while the new bag recorded 64 clouds; the cause remains open.

See the [validation index](DRONE_2D_MAPPING_VALIDATION.md) for the latest status,
limits and remaining work. Representative outputs are now included in
[curated evidence](results/drone-bird-map/20261031-validation/README.md).
Earlier references to local-only evidence remain historical; raw bags/full logs
are still local-only. This update does not claim those datasets are in Git.
