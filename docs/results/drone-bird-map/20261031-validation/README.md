# Curated drone mapping evidence — 2026-10-31

This directory contains a small selection of measured results, not the complete
raw experiment archive. See [validation index](../../../DRONE_2D_MAPPING_VALIDATION.md)
for conclusions, reproduction requirements and outstanding work.

| Directory | Evidence | Important boundary |
|---|---|---|
| window_rule | Formal/offline rule comparison and representative image | Retention is not ground-truth accuracy |
| window_motion | Exported PGM/YAML, preview, route and reload result | Earlier A map/statistics pair was mismatched; final export passed |
| wall_scale | Wall fit results and selected regions | Candidate association; no whole-map acceptance |
| landmarks | Old/new fit summaries, absent-wall result, sampled truth and paired statistics | Includes insufficient-support outcomes; not all tests passed |

`window_motion/drone_bird_map.yaml` and its PGM must stay together. This is the
19:36 bounded-motion map, not a map exported after the 20:43 stationary capture.
The later capture contains matched statistics and reports but no new final PGM
in this curated set. Images are previews; they do not replace raw grid arrays.

`manifest.json` records exact file hashes and original paths relative to the
local `mapping-test-20261031` archive. Original reports are copied unchanged;
some contain the author's local absolute paths. These are provenance labels,
not portable input paths or files included in this repository.

Raw bags, raw cloud arrays, full entity-state dumps and full per-frame logs remain
in external/local storage. Ask the experiment author for those files to reproduce
the numerical analyses. The summaries alone cannot independently prove every
reported result. Do not claim absent raw evidence is committed.
