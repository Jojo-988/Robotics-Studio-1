#!/usr/bin/env python3
"""Project one saved camera sample onto a horizontal plane.

Offline tool: NumPy + Pillow + PyYAML. No ROS runtime needed.
Output image: right = +X, up = +Y in the target frame.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image
import yaml


from map_data.camera_projection import require, read_json, project_rgb, self_test


def project(args):
    sample = args.sample.expanduser().resolve()
    output = args.output.expanduser().resolve()

    require(not output.exists(), "Output already exists; choose a new directory")
    require(
        math.isfinite(args.resolution) and args.resolution > 0,
        "Resolution must be finite and positive",
    )
    require(math.isfinite(args.ground_z), "Invalid ground height")
    require(0 <= args.exclude_bottom < 1, "exclude-bottom must be in [0,1)")

    capture = read_json(sample / "validation.json")
    require(capture["status"] == "PASS", "Capture validation did not pass")

    meta = read_json(sample / "image_metadata.json")
    pose = read_json(sample / "pose.json")
    info = yaml.safe_load(
        (sample / "camera_info.yaml").read_text(encoding="utf-8")
    )

    with Image.open(sample / "image.png") as image:
        require(image.mode == "RGB", "This version expects an RGB PNG")
        image.load()
        rgb = np.frombuffer(
            image.tobytes(), dtype=np.uint8
        ).reshape(image.height, image.width, 3).copy()

    rgba, mask, metadata = project_rgb(
        rgb, info, pose, meta, args.resolution, args.ground_z,
        args.exclude_bottom, source_sample=sample)
    out_height, out_width = mask.shape
    resolution = args.resolution
    probe_world = [metadata['principal_pixel_ground_xyz']]
    error = metadata['internal_roundtrip_max_error_px']
    output.mkdir(parents=True, exist_ok=False)
    Image.fromarray(rgba).save(output / "bird_view.png")
    Image.fromarray(mask).save(output / "valid_mask.png")
    (output / "metadata.json").write_text(
        json.dumps(metadata, indent=2, allow_nan=False), encoding="utf-8"
    )

    print(f"Saved: {output}")
    print(f"Output: {out_width} x {out_height}, {resolution} m/pixel")
    print(f"Principal pixel ground XYZ: {probe_world[0]}")
    print(f"Valid projected area: {metadata['valid_area_m2']:.4f} m^2")
    print(f"Internal round-trip error: {error:.3e} pixels")
    print("PASS: projection completed; map accuracy is NOT yet validated")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sample", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--resolution", type=float, default=0.01)
    parser.add_argument("--ground-z", type=float, default=0.0)
    parser.add_argument("--exclude-bottom", type=float, default=0.10)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    if args.self_test:
        self_test()
        return

    if args.sample is None or args.output is None:
        parser.error("--sample and --output are required")
    project(args)


if __name__ == "__main__":
    main()