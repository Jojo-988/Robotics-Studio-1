# Retired drone LiDAR 2D prototype

Archived on 2026-11-01 after the drone bird-eye deliverable changed to camera mapping.
These results are historical evidence, not current camera-map accuracy results.

- Current interface: [Camera mapping](../../CAMERA_2D_MAPPING.md).
- GUI guide: [Camera GUI handoff](../../CAMERA_GUI_HANDOFF.md).
- Historical validation: [Evidence index](docs/DRONE_2D_MAPPING_VALIDATION.md).
- Offline LiDAR tools: [Tools](tools/drone_mapping/README.md).

The old runtime and tests can be inspected at commit `e1e091e`. Do not revert that
entire commit: it also contains the shared drone localization correction.
Old command examples assume that historical checkout and its original paths.
Archived standalone tool tests can still be run from the current repository root:

```bash
python3 -m unittest discover -s docs/archive/drone-lidar-2d/tools/drone_mapping -p 'test_*.py' -v
```

Team LiDAR handler, reconstruction, 3D mapping, Husky statistics and camera runtime
remain active. Clients of the retired /parrot1/bird_map OccupancyGrid interface must
migrate explicitly to /parrot1/camera_bird_map image/mask/metadata; types differ.
