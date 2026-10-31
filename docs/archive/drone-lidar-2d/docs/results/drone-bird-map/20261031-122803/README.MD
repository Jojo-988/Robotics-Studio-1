> **Archived 2026-11-01 — retired drone LiDAR 2D prototype.** Commands and results below are historical; removed runtime code is available at Git commit `e1e091e`. Use the current camera mapping documentation for GUI integration.

# Saved drone bird's-eye grid: 2026-10-31 12:28:03

This directory contains an earlier simple-scene prototype snapshot and the
messages used to verify its export/reload. It is not the later forest map,
the 17:17:55 route trial or the offline window-rule experiment.

## Contents

| File | Purpose |
|---|---|
| `drone_bird_map.pgm` | Saved classification image |
| `drone_bird_map_fixed.yaml` | Reload metadata; image path is relative to this directory |
| `before_save.yaml` | Full source ROS OccupancyGrid message |
| `after_reload_fixed.yaml` | Full independently reloaded ROS OccupancyGrid message |
| `README.MD` | Sample description and verification procedure |

No RViz screenshot is included in this directory. Both message files use ROS
CLI YAML output with document separators; read them with `yaml.safe_load_all`.

## Geometry and class counts

- Frame: `parrot1_odom`.
- Resolution: 0.1 m/cell (ROS float representation is approximately 0.10000000149).
- Dimensions: 200 x 200 cells; total 40000.
- Origin: (-10,-10,0) m; identity orientation.
- Extent: approximately [-10,10) m on each horizontal axis.

| Class | Before save | After reload |
|---|---:|---:|
| Unknown (-1) | 33578 | 33578 |
| Ground candidate (0) | 6341 | 6341 |
| Structure candidate (100) | 81 | 81 |

Rechecked during documentation review: matching frame, matching geometry
(absolute tolerance 1e-6, relative tolerance 0), and **0 changed cells**.
Message and map-load timestamps are deliberately not compared.

## Reproduce the cell comparison

Run the following from this directory in a Python environment with NumPy and
PyYAML installed. ROS, Gazebo and RViz do not need to be running.

```bash
python3 - <<'PY'
from pathlib import Path
from collections import Counter
import numpy as np
import yaml

def load(name):
    with Path(name).open() as stream:
        return next(d for d in yaml.safe_load_all(stream)
                    if isinstance(d, dict) and 'data' in d)

def geometry(message):
    i = message['info']
    p = i['origin']['position']
    q = i['origin']['orientation']
    return [i['width'], i['height'], i['resolution'],
            p['x'], p['y'], p['z'], q['x'], q['y'], q['z'], q['w']]

before = load('before_save.yaml')
after = load('after_reload_fixed.yaml')
for message in (before, after):
    assert len(message['data']) == message['info']['width'] * message['info']['height']
assert len(before['data']) == len(after['data'])
same_frame = before['header']['frame_id'] == after['header']['frame_id']
same_geometry = np.allclose(geometry(before), geometry(after), atol=1e-6, rtol=0)
changed = np.count_nonzero(np.asarray(before['data']) != np.asarray(after['data']))
print('Before counts:', dict(Counter(before['data'])))
print('After counts:', dict(Counter(after['data'])))
print('Same frame:', same_frame)
print('Same geometry:', bool(same_geometry))
print('Changed cells:', int(changed))
assert same_frame and same_geometry and changed == 0
print('PASS')
PY
```

## Reload settings and interpretation

Keep the PGM and YAML together and load `drone_bird_map_fixed.yaml`.
It specifies `mode: trinary`, `negate: 0`, `free_thresh: 0.15`, and
`occupied_thresh: 0.65`. These settings preserve the unknown pixels in this
sample's encoding. A previous threshold setting misclassified unknown as free;
that failed reload is not the `after_reload_fixed.yaml` retained here.

White represents ground candidates, black represents structure candidates,
and grey represents unknown/unresolved evidence. The map uses simulation-assisted
pose and is not a colour aerial image or validated navigation map.
The comparison establishes preservation of the saved classes and geometry,
not absolute mapping accuracy, complete survey coverage or traversability.
The PGM/YAML pair does not contain historical observation counts and cannot
resume the accumulation session.
