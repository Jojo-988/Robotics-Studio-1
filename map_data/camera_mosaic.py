"""Bounded world-aligned RGB mosaic. New valid pixels replace old pixels."""
import numpy as np


class CameraMosaic:
    def __init__(self, resolution, origin_x, origin_y, width, height):
        values = np.array([resolution, origin_x, origin_y], dtype=float)
        if not np.isfinite(values).all() or resolution <= 0:
            raise ValueError('Invalid resolution/origin')
        if width <= 0 or height <= 0 or width * height > 4_000_000:
            raise ValueError('Canvas must contain 1..4,000,000 pixels')
        origin = np.array([origin_x, origin_y]) / resolution
        if not np.allclose(origin, np.rint(origin), atol=1e-6, rtol=0):
            raise ValueError('Origin must align to resolution')
        self.resolution = resolution
        self.lower = np.rint(origin).astype(int)
        self.width, self.height = width, height
        self.rgba = np.zeros((height, width, 4), dtype=np.uint8)

    def clear(self):
        self.rgba.fill(0)

    def add(self, rgba, mask, metadata):
        if abs(metadata['resolution_m_per_pixel'] - self.resolution) > 1e-12:
            raise ValueError('Resolution mismatch')
        if rgba.shape[:2] != mask.shape or rgba.shape[2] != 4:
            raise ValueError('Tile shapes differ')
        if not np.array_equal(mask > 0, rgba[:, :, 3] > 0):
            raise ValueError('Tile alpha/mask mismatch')
        bounds = metadata['bounds']
        indexes = np.array([bounds['xmin'], bounds['ymax']])/self.resolution
        if not np.allclose(indexes, np.rint(indexes), atol=1e-6, rtol=0):
            raise ValueError('Tile is not world-grid aligned')
        x0 = int(round(indexes[0])) - self.lower[0]
        y0 = int(self.lower[1] + self.height - round(indexes[1]))
        hh, ww = mask.shape
        left, top = max(0, x0), max(0, y0)
        right, bottom = min(self.width, x0+ww), min(self.height, y0+hh)
        total = int(np.count_nonzero(mask))
        if left >= right or top >= bottom:
            return 0, total
        tile = rgba[top-y0:bottom-y0, left-x0:right-x0]
        selected = mask[top-y0:bottom-y0, left-x0:right-x0] > 0
        destination = self.rgba[top:bottom, left:right]
        destination[selected] = tile[selected]
        inside = int(selected.sum())
        return inside, total-inside
