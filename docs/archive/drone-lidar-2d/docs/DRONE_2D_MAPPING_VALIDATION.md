> **Archived 2026-11-01 — retired drone LiDAR 2D prototype.** Commands and results below are historical; removed runtime code is available at Git commit `e1e091e`. Use the current camera mapping documentation for GUI integration.

# Drone mapping validation and evidence index

Status: 2026-10-31. The system is a simulation-assisted LiDAR XY classification
mapper. The following checks support a working prototype; they do not establish
complete forest coverage, navigation safety or independent SLAM accuracy.

## Reading order

1. [Mapping interface and operation](DRONE_2D_MAPPING.md).
2. [Chronological test log](DRONE_2D_MAPPING_TEST_LOG.md).
3. [Curated evidence](results/drone-bird-map/20261031-validation/README.md).
4. [Offline tools](../tools/drone_mapping/README.md).

The Husky trail report [quality_check.md](../../../quality_check.md) describes a separate
robot/map and must not be treated as drone mapping evidence.

## Results and limits

| Check | Observed result | Interpretation |
|---|---|---|
| Window confirmation | Formal module matched all 53 offline reference frames; final masks 774 cumulative vs 569 window cells | Implementation agreement, not truth-labelled precision |
| Reset | Window fill/counts cleared; map became unknown and repopulated | Reset integration observed |
| Paired snapshots | Exact source stamp plus count checks; old A mismatch rejected | Prevents mixing map and statistics from different frames |
| Bounded forest movement | A→B→A; odometry return error 0.010895 m | Short-motion evidence, not independent map positioning error |
| Export/reload | Final bounded-motion grid: zero changed cells | Class preservation, not geometric accuracy |
| Wall scale | Left/right spacing approximately 25.0 m vs nominal 25 m | No obvious scale distortion in selected walls |
| Tree, old bag | 52/53 supported fits; median per-frame XY deviation 0.0203 m | Conditional candidate fit; no contemporaneous entity truth |
| Bottom wall, old bag | No returns or confirmed cells in selected segment | Insufficient support, not a failed positioning measurement |
| Tree, new capture | 58/64 supported fits; median XY deviation 0.0206 m, radius 0.1728 m vs 0.18 m | Raw point-cloud local fit, not grid/global accuracy |
| Right lower wall, new capture | 1600 returns; p95 absolute distance 0.0409 m | Held-out segment from the same wall family; no fitted alignment correction |
| Wall in accumulated grid | 47 cells, centre distance approximately 0.05 m | Consistent with half-cell quantisation; includes earlier map history |
| Animal candidate | Only 3–4 points/frame after nose exclusion | Insufficient support; no position acceptance |

New capture: `landmark-acquisition-20261031-204320`. Gazebo ran briefly at the
existing position and was paused afterwards. Recorded entity poses for drone,
tree, animal and the inspected wall were unchanged across periodic samples.
The raw odometry position was also constant. These samples are not per-cloud
synchronised truth and do not independently validate every instant.

The current forest map has 0.1 m cells. Centimetre-level raw-cloud fitting must
not be advertised as centimetre-level accuracy of the complete grid. Thresholds
used to reject poor geometric fits are tool diagnostics, not agreed acceptance
criteria. No acceptance tolerance or task-area polygon has yet been established.

## Remaining work

- Define the target area and required accuracy with the team.
- Validate bounded multi-view/turning acquisition and structure alignment.
- Inspect persistent false structures; do not replace unknown cells with free space.
- Explain the new bag's 64 received cloud frames versus the mapper's 33 accepted
  frame increase (547→580). Zero TF/busy counters do not explain this difference.
- Export the final task map and repeat paired snapshot and cellwise reload checks.

## Reproduction and storage

The repository includes representative outputs and reusable geometry tools.
The raw recordings remain in local/external experiment storage. The earlier
stationary recording is `diagnostic-20261031-174614/stationary_cloud`; the newer
recording is `landmark-acquisition-20261031-204320/bag` in the local archive.
Request those datasets before attempting to reproduce bag-derived numbers.

Run tests from the repository root, after sourcing ROS Humble where needed:

```bash
python3 -m unittest discover -s test -p 'test_bird_grid.py' -v
python3 -m unittest discover -s test -p 'test_structure_window.py' -v
python3 -m unittest discover -s test -p 'test_snapshot_pairing.py' -v
python3 -m unittest discover -s tools/drone_mapping -p 'test_*.py' -v
```

Keep source, config, tests, documentation and a small set of results in Git.
Keep rosbag files, generated build/install/log directories, `.bak-*` files,
full point-cloud dumps and duplicate screenshots outside Git. No commit/push
is performed by this documentation organisation step.
