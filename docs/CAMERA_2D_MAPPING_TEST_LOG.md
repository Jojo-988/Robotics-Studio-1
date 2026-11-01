# Camera mapping test log

Recorded 2026-11-01 (Australia/Sydney). Saved-map directory names use UTC;
20261031T14...Z and T15...Z belong to these local 2026-11-01 experiments.

## Scope and setup

Camera ground-plane mosaic, simulation pose at image time; ground_z=0,
resolution=0.01 m/pixel, canvas=800x800, origin=(-2,-4), target=parrot1_odom,
bottom exclusion=0.10, latest_valid_pixel_wins. Body z=0.8 m; heading unchanged.
No independent world-position calibration, terrain reconstruction, whole-forest
coverage or non-ground-object accuracy is claimed.

Evidence is from movement JSON, per-stage metadata and saved PNG/mask/JSON files.
Raw experiment files remain in /home/zora/team-results/drone-camera on Zora's
machine; the paths below are provenance, not files shipped by this document.

## Forward strip: PASS for bounded translation and coverage

Run: strip-20261101-011805. Files move_01.json through move_04.json and
metadata_01.txt through metadata_04.txt.
Epoch: ea213cf9-2ad1-44f5-81f1-d3cb0f2e986f throughout.

| Stage | End body X (m) | Valid pixels | Valid area (m2) |
|---|---:|---:|---:|
| Start | 2.530 | Not captured as a separate baseline | — |
| 1 | 2.794 | 36450 | 3.6450 |
| 2 | 3.058 | 42300 | 4.2300 |
| 3 | 3.322 | 48150 | 4.8150 |
| 4 | 3.586 | 54225 | 5.4225 |

All movement reports returned TARGET_REACHED. Total displacement=1.056 m,
y=0, z=0.8, identity orientation. Each metadata source time is later than its
movement end. Reported unmatched_images, tf_timeout, invalid and outside_frames
were zero. superseded_pairs counts intentional replacement of older input pairs;
it is not included in the zero-error claim.

Saved: live/20261031T142953_365739Z_ea213cf9.
Version=605; source_stamp=10392.940000000; processed_frames=604.
COMPLETE exists; RGBA alpha equals mono8 mask exactly; image dimensions and
54225 valid pixels match metadata. Mask bbox [418,287,659,512] (exclusive upper
bounds) is a full 241x225 rectangle with zero unknown pixels inside it.
Ground/trail texture appears continuous at inspected resolution. A raised,
tree-like feature remains distorted; plane projection does not validate it.

## Lateral move: PASS for +Y expansion and local image consistency

Run: lateral-20261101-020316; move_left_01.json.
Start=(3.586,0,0.8); end=(3.586,0.264,0.8), unchanged orientation.
Actual progress=0.264 m; cross-track error approximately 1.75e-13 m (numerical
round-off, not a real-world accuracy measurement). TARGET_REACHED.
End source time=10450.610000000.

Saved: live/20261031T150430_803663Z_ea213cf9.
Version=671; source_stamp=10461.530000000; processed_frames=670.
Same epoch, geometry and frame as forward-strip save; COMPLETE and alpha/mask
consistency checks passed. Valid pixels=57735; area=5.7735 m2.

| Comparison | Measured result |
|---|---|
| Old valid pixels lost | 0 |
| New valid pixels | 3510 |
| New area | 0.351 m2 |
| Before bbox | [418,287,659,512] |
| After bbox | [418,261,659,512] |
| New-pixel bbox | [524,261,659,287] |

All new pixels form a 135x26 strip above the right-hand part of the previous
coverage. Up corresponds to +Y. The 26-pixel extension is consistent with
0.264/0.01=26.4 pixels after raster quantization. The whole old map must not move.

Ground texture checks compared unmodified saved maps at their shared coordinates.
Search: integer shifts dx,dy in [-5,5], mean absolute RGB difference, no registration
applied. ROIs are pixel rectangles [x0,y0,x1,y1], excluding the raised feature.

| ROI | Pixel rectangle | Best shift (px) | RGB MAE (0–255) | Changed pixels |
|---|---|---|---:|---:|
| Grass | [540,305,630,370] | (0,0) | 0.2242 | 3043 |
| Trail | [550,390,620,440] | (0,0) | 0.2266 | 1824 |

Both regions contain updated pixels. No integer-pixel translational misalignment
was detected in these selected ground regions. This is local consistency evidence,
not independent metric accuracy or a subpixel error bound. Common-area RGB MAE
was 0.3403; old unchanged pixels also contribute to that overall value.

## Backend save, reset and resume: PASS (2026-11-01)

These are backend service/data checks, not team GUI acceptance. No movement was
needed. Two successive reset experiments are recorded separately below.

### Run 1: saved artifact, reset metadata and resumed acquisition

Evidence directory: /home/zora/team-results/drone-camera/backend-check-20261101-022537.
Evidence: save_response.txt, saved_directory.txt, before_reset.txt,
after_reset.txt, after_resume.txt and after_resume_retry.txt.

Save returned success and created:
/home/zora/team-results/drone-camera/live/20261031T152713_781359Z_01028b8b.
The save check passed: COMPLETE exists; bird_view.png and valid_mask.png dimensions
match metadata, alpha equals mask, and mask count/area agree with metadata.
Saved version=49, source time=65.940000000, processed_frames=48;
valid_pixels=30375, valid_area_m2=3.0375. Canvas remains 800x800 at 0.01 m/pixel.

Reset changed epoch from 01028b8b-05b3-4451-81eb-0617ed0d8f30 to
1775059f-d060-40c3-b0fa-bc5b7275b53e. At source time=66.050000000,
status=reset, processed_frames=0, valid_pixels=0 and area=0.
The previously saved artifact remained present.

The initial resume check FAILED: after_resume.txt still contained the same
retained reset state (time=66.050000000). This did not demonstrate resumed input.
After retry, after_resume_retry.txt showed status=updated in the same new epoch,
source time=139.870000000, version=72, processed_frames=71, valid_pixels=30375
and area=3.0375. Resume therefore passed on retry. The records alone do not
distinguish a still-paused simulator from reading retained reset data too early.
Unmatched-image, TF-timeout, invalid and outside-frame counters were zero;
superseded_pairs=74 is reported separately and is not a zero-error claim.

### Run 2: timestamp-paired image/mask clearing

Evidence directory: /home/zora/team-results/drone-camera/clear-check-20261101-024252.
Files: check_camera_clear_snapshot.py, before.json, reset_response.txt,
after_reset.json. The checker is read-only; the reset was called separately.
Gazebo was paused for the reset-state inspection.

| Field | Before | After reset |
|---|---|---|
| Check result | BEFORE SNAPSHOT PASS | RESET SNAPSHOT PASS |
| Epoch | 1775059f-d060-40c3-b0fa-bc5b7275b53e | e5614893-c8d1-4712-8fd1-7d5a40c4d52d |
| Shared output stamp | 283.060000000 | 283.130000000 |
| Status | updated | reset |
| Processed frames | 206 | 0 |
| Valid mask pixels | 30375 | 0 |
| Nonzero RGB channel values | 91125 | 0 |
| Valid area (m2) | 3.0375 | 0 |

Each snapshot pairs image.header.stamp and valid_mask.header.stamp with
metadata.output_stamp using exact integer seconds/nanoseconds. Frame and
dimensions match; mask values are binary; RGBA alpha equals mask pixel-for-pixel;
counts and area match metadata. After reset RGB, alpha and mask are all zero.
Frame, dimensions, origin and resolution were unchanged. This establishes
coherent clearing of the three published outputs, not simultaneous network arrival.
Resume was demonstrated in Run 1; no separate post-Run-2 resume evidence is claimed.

### Handoff readiness

Backend publication, saved artifacts, reset clearing and resumed acquisition are
ready for GUI integration. Team GUI display, timestamp pairing, Save/Reset buttons,
UI responsiveness and late-connection behavior remain Pending. These tests do not
establish whole-scene coverage, independent map accuracy or navigability.

## GUI acceptance: NOT YET RUN

Local GUI_Integration/GUI_Test2.py does not yet subscribe to camera_bird_map
outputs or call camera_bird_mapper save/reset. rqt image display is backend
visualization evidence only. The teammate's unpushed code has not been inspected.

| Check | Expected | Status |
|---|---|---|
| Late GUI connection | Complete retained image/mask/metadata set | Pending |
| Display | RGBA transparency, correct orientation and no stretching | Pending |
| Metadata | Matched output_stamp; correct area/epoch/version | Pending |
| GUI Save button | Async Trigger call, returned path, COMPLETE | Pending |
| GUI Reset button | New epoch, empty image/mask/area while Gazebo paused | Pending |
| Resume | Same new epoch repopulates from fresh images | Pending |
| Pause | Responsive GUI, no false claim of new frames | Pending |

Use [GUI handoff](CAMERA_GUI_HANDOFF.md) and [interface](CAMERA_2D_MAPPING.md).
Save before reset. During pause, simulation time may stop: source_stamp age alone
cannot detect lack of arrivals. GUI should also track wall-monotonic time since
the last completed frame set, and distinguish paused/no-update from a known error.
No GUI acceptance result should be marked PASS based solely on a terminal call.


## 15 m patrol and offline target review — 2026-11-01

Evidence: [machine-readable review](results/camera-height15-20261101/review.json),
[before/after target](results/camera-height15-20261101/target_comparison.png).
The ground comparison image and extended delivery notes remain local; the numerical
ROI results are included in review.json and summarized below.

User-provided patrol logs report COMPLETE, 16/16 waypoints, no health holds, and
zero cmd_vel publishers after clean exit. Saved maps have the same geometry and
epoch. Offline checks verified COMPLETE markers, binary masks, alpha/mask equality
and metadata pixel counts. Old valid pixels lost: 0. Target sampling increased
94% to 100%, adding 37.5 m2 within the selected 25 x 25 m target. The target assumes
the world-origin forest plane and world/odom XY alignment, not surveyed ground truth.

Two previously selected exposed-ground candidate patches have best integer shifts
(0,0) in a +/-5 pixel RGB-MAE search, with MAE 2.1454 and 2.1251 on a 0..255 scale.
958 and 833 pixels changed, respectively. Visual inspection found no obvious local
double edges. This does not establish subpixel or whole-map absolute accuracy.
Trees, raised objects and roof appearance still differ; planar parallax and scene
changes cannot be separated using these two mosaics alone.

Saved endpoint tf_timeout, invalid and outside_frames counts are zero. This is
endpoint evidence, not a continuous frame-by-frame audit. Full-canvas new pixels
include areas outside the forest target; mask coverage is not visible-ground or
navigability coverage. The result supports an overview prototype, not an
orthorectified survey map. Team GUI integration acceptance remains a separate test.

The earlier GUI acceptance section records the state at that earlier inspection;
it is not a claim about the teammate's latest GUI implementation.


## Offline projection overlay regression tests

The reusable comparison command remains in scripts/compare_camera_projections.py.
Its synthetic regression tests now live in test/test_projection_overlay.py as
unittest.TestCase tests. Importing the test module does not execute comparisons.
The test resolves the comparison tool from the repository root, not its own directory.

Run from the repository root (Python dependencies: NumPy and Pillow; no simulation needed):

```bash
python3 -m unittest discover -s test -p 'test_projection_overlay.py' -v
```

Tests cover world-coordinate placement, overlap count, identical overlapping
texture, generated comparison output, and explicit rejection of the same input
sample. They do not validate physical camera calibration or real-scene accuracy.
Commit the test together with scripts/compare_camera_projections.py; the old
scripts/test_projection_overlay.py entry point has been retired.
