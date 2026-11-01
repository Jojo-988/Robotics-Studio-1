#!/usr/bin/env python3
"""Place A/B projections on one world-aligned raster, without registration."""
import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image


def require(condition, message):
    if not condition:
        raise ValueError(message)


def read_projection(folder):
    meta = json.loads((folder / 'metadata.json').read_text())
    with Image.open(folder / 'bird_view.png') as im:
        rgba = np.frombuffer(im.convert('RGBA').tobytes(), dtype=np.uint8)
        rgba = rgba.reshape(im.height, im.width, 4).copy()
    with Image.open(folder / 'valid_mask.png') as im:
        mask = np.frombuffer(im.convert('L').tobytes(), dtype=np.uint8)
        mask = mask.reshape(im.height, im.width).copy()
    shape = (meta['height'], meta['width'])
    require(rgba.shape[:2] == shape and mask.shape == shape, 'Image size mismatch')
    require(np.isin(mask, [0, 255]).all(), 'Mask must be binary')
    require(np.array_equal(rgba[:, :, 3] > 0, mask > 0), 'Alpha/mask mismatch')
    require(meta['image_axes'] == 'right=+X, up=+Y', 'Unsupported image axes')
    require(abs(meta['origin_lower_left']['yaw']) < 1e-9, 'Nonzero grid yaw')
    return meta, rgba[:, :, :3], mask > 0


def grid_bounds(meta, res):
    b = meta['bounds']
    values = np.array([b[k] for k in ('xmin', 'ymin', 'xmax', 'ymax')]) / res
    require(np.isfinite(values).all(), 'Invalid bounds')
    rounded = np.rint(values).astype(int)
    require(np.allclose(values, rounded, atol=1e-6, rtol=0), 'Unaligned grid')
    x0, y0, x1, y1 = rounded
    require(x1-x0 == meta['width'] and y1-y0 == meta['height'], 'Bounds mismatch')
    return rounded


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--a', required=True, type=Path)
    parser.add_argument('--b', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    require(not args.output.exists(), 'Output exists; choose a new directory')
    ma, a, am = read_projection(args.a)
    mb, b, bm = read_projection(args.b)
    require(ma['frame_id'] == mb['frame_id'], 'Frames differ')
    require(ma['source_stamp'] != mb['source_stamp'], 'Same image timestamp')
    res = ma['resolution_m_per_pixel']
    require(np.isfinite(res) and res > 0, 'Invalid resolution')
    require(abs(res-mb['resolution_m_per_pixel']) < 1e-12, 'Resolutions differ')
    require(abs(ma['ground_z']-mb['ground_z']) < 1e-9, 'Ground planes differ')
    require(ma['optical_convention'] == mb['optical_convention'], 'Conventions differ')
    require(ma['exclude_bottom_fraction'] == mb['exclude_bottom_fraction'], 'Masks differ')
    ba, bb = grid_bounds(ma, res), grid_bounds(mb, res)
    xmin, ymin = np.minimum(ba[:2], bb[:2])
    xmax, ymax = np.maximum(ba[2:], bb[2:])
    w, h = int(xmax-xmin), int(ymax-ymin)
    require(0 < w*h <= 4_000_000, 'Canvas too large')

    def place(rgb, mask, bounds):
        canvas = np.zeros((h, w, 3), dtype=np.uint8)
        valid = np.zeros((h, w), dtype=bool)
        col = int(bounds[0]-xmin)
        row = int(ymax-bounds[3])
        hh, ww = mask.shape
        canvas[row:row+hh, col:col+ww] = rgb
        valid[row:row+hh, col:col+ww] = mask
        return canvas, valid, [col, row]

    a, am, offset_a = place(a, am, ba)
    b, bm, offset_b = place(b, bm, bb)
    overlap, union = am & bm, am | bm
    require(overlap.any(), 'No overlapping valid pixels')

    def rgba(rgb, valid):
        return np.dstack((rgb, valid.astype(np.uint8)*255))

    blend = a.copy()
    blend[bm & ~am] = b[bm & ~am]
    blend[overlap] = np.rint(
        (a[overlap].astype(float) + b[overlap])/2
    ).astype(np.uint8)
    rows, cols = np.indices((h, w))
    choose_b = ((rows//20 + cols//20) % 2 == 1)
    checker = a.copy()
    use_b = bm & (~am | choose_b)
    checker[use_b] = b[use_b]
    delta = np.abs(a.astype(float)-b.astype(float)).mean(axis=2)
    difference = np.zeros((h, w, 3), dtype=np.uint8)
    difference[overlap] = np.rint(delta[overlap, None]).astype(np.uint8)
    coverage = np.zeros((h, w, 3), dtype=np.uint8)
    coverage[am & ~bm] = [255, 0, 0]
    coverage[bm & ~am] = [0, 0, 255]
    coverage[overlap] = [255, 255, 255]

    args.output.mkdir(parents=True, exist_ok=False)
    images = {
        'a_aligned.png': rgba(a, am), 'b_aligned.png': rgba(b, bm),
        'overlay_50.png': rgba(blend, union),
        'checkerboard.png': rgba(checker, union),
        'difference.png': rgba(difference, overlap),
        'coverage.png': coverage,
    }
    for name, data in images.items():
        Image.fromarray(data).save(args.output / name)
    # Blink only the overlap, avoiding distracting changes at coverage edges.
    af, bf = np.full_like(a, 70), np.full_like(b, 70)
    af[overlap], bf[overlap] = a[overlap], b[overlap]
    Image.fromarray(af).save(
        args.output / 'blink.gif', save_all=True,
        append_images=[Image.fromarray(bf)], duration=700, loop=0)
    report = {
        'status': 'COMPARISON_READY_NOT_ACCURACY_PASS',
        'a': str(args.a.resolve()), 'b': str(args.b.resolve()),
        'frame_id': ma['frame_id'], 'resolution': res,
        'canvas_width': w, 'canvas_height': h,
        'origin_lower_left': [float(xmin*res), float(ymin*res)],
        'offset_a_column_row': offset_a, 'offset_b_column_row': offset_b,
        'camera_delta_xyz': (np.array(mb['camera_optical_center'])
                             - np.array(ma['camera_optical_center'])).tolist(),
        'overlap_pixels': int(overlap.sum()),
        'overlap_area_m2': float(overlap.sum()*res*res),
        'overlap_fraction_of_smaller_valid_image': float(
            overlap.sum()/min(am.sum(), bm.sum())),
        'overlap_mean_absolute_rgb_difference_0_255': float(delta[overlap].mean()),
        'registration_applied': False,
        'note': 'Photometric difference is not geometric error. Inspect flat ground; '
                'raised objects, shadows and texture rendering may differ. '
                'Matching frame names do not prove the same simulation session.',
    }
    (args.output / 'comparison.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    print('Saved comparison images to', args.output)


if __name__ == '__main__':
    main()
