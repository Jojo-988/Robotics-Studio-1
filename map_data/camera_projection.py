"""Pure projection math shared by offline and live camera mapping."""
import json
import math
from types import SimpleNamespace
import numpy as np

def require(condition, message):
    if not condition:
        raise ValueError(message)


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def stamp_key(stamp):
    return int(stamp["sec"]), int(stamp["nanosec"])


def quaternion_matrix(rotation):
    q = np.array(
        [rotation[name] for name in ("x", "y", "z", "w")],
        dtype=float,
    )
    require(np.isfinite(q).all(), "Non-finite quaternion")
    norm = np.linalg.norm(q)
    require(abs(norm - 1.0) < 0.001, "Quaternion is not normalized")
    x, y, z, w = q / norm

    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def pixels_to_plane(uv, k, rotation, center, ground_z):
    """Intersect optical pixel rays with z=ground_z.

    rotation maps optical coordinates into the target frame.
    """
    uv = np.atleast_2d(np.asarray(uv, dtype=float))
    pixels = np.column_stack((uv, np.ones(len(uv))))
    optical_rays = pixels @ np.linalg.inv(k).T
    world_rays = optical_rays @ rotation.T

    require(
        np.all(world_rays[:, 2] < -1e-8),
        "Some image rays are horizontal/upward; this first version "
        "requires the complete image footprint to face the ground.",
    )

    distance = (ground_z - center[2]) / world_rays[:, 2]
    require(np.all(distance > 0), "Ground intersections are behind camera")
    return center + distance[:, None] * world_rays


def world_to_pixels(points, k, rotation, center):
    """Project target-frame points into optical pixels."""
    optical = (np.asarray(points) - center) @ rotation
    depth = optical[:, 2]
    homogeneous = optical @ k.T

    uv = np.full((len(optical), 2), np.nan)
    positive = depth > 1e-8
    uv[positive] = (
        homogeneous[positive, :2] / homogeneous[positive, 2:3]
    )
    return uv, depth


def self_test():
    """Synthetic geometry tests; these do not validate the real camera."""
    k = np.array([
        [200.0, 0.0, 360.0],
        [0.0, 200.0, 240.0],
        [0.0, 0.0, 1.0],
    ])

    # Camera link is pitched 90 degrees downward.
    link_rotation = np.array([
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
    ])

    # Optical x=right, y=down, z=forward -> link x=forward,y=left,z=up.
    optical_to_link = np.array([
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ])

    rotation = link_rotation @ optical_to_link
    center = np.array([2.25, 0.0, 0.65])
    pixels = np.array([
        [360.0, 240.0],
        [460.0, 240.0],
        [360.0, 340.0],
    ])

    points = pixels_to_plane(pixels, k, rotation, center, 0.0)
    expected = np.array([
        [2.25, 0.0, 0.0],
        [2.25, -0.325, 0.0],
        [1.925, 0.0, 0.0],
    ])
    np.testing.assert_allclose(points, expected, atol=1e-10)

    recovered, _ = world_to_pixels(points, k, rotation, center)
    np.testing.assert_allclose(recovered, pixels, atol=1e-10)

    # Also check composition with a nonzero map yaw.
    angle = 0.3
    yaw = np.array([
        [math.cos(angle), -math.sin(angle), 0.0],
        [math.sin(angle), math.cos(angle), 0.0],
        [0.0, 0.0, 1.0],
    ])
    rotated = pixels_to_plane(
        pixels, k, yaw @ rotation, center, 0.0
    )
    np.testing.assert_allclose(
        rotated - center, (points - center) @ yaw.T, atol=1e-10
    )

    try:
        pixels_to_plane(pixels, k, np.eye(3), center, 0.0)
    except ValueError:
        pass
    else:
        raise AssertionError("Upward rays should have been rejected")

    print("SELF TEST PASS: center, axes, scale, round-trip, yaw, rejection")


def project_rgb(rgb, info, pose, meta, resolution=0.01, ground_z=0.0,
                exclude_bottom=0.10, source_sample="live"):
    """Return RGBA, binary mask and geometry metadata; no ROS or file writes."""
    args = SimpleNamespace(resolution=resolution, ground_z=ground_z,
                           exclude_bottom=exclude_bottom)
    require(np.isfinite(resolution) and resolution > 0, "Invalid resolution")
    require(np.isfinite(ground_z), "Invalid ground height")
    require(0 <= exclude_bottom < 1, "Invalid bottom exclusion")
    require(rgb.dtype == np.uint8 and rgb.ndim == 3 and rgb.shape[2] == 3,
            "Expected uint8 RGB image")
    height, width = rgb.shape[:2]
    require(
        (width, height) == (info["width"], info["height"])
        == (meta["width"], meta["height"]),
        "Image dimensions differ from metadata",
    )

    tf = pose["transform"]
    stamps = [
        meta["header"]["stamp"],
        info["header"]["stamp"],
        pose["image_stamp"],
        pose["camera_info_stamp"],
        tf["header"]["stamp"],
    ]
    require(
        len({stamp_key(s) for s in stamps}) == 1,
        "Image, CameraInfo and pose timestamps do not match",
    )
    require(
        meta["header"]["frame_id"] == info["header"]["frame_id"]
        == pose["source_frame"] == tf["child_frame_id"],
        "Source frame IDs do not match",
    )
    require(
        tf["header"]["frame_id"] == pose["target_frame"],
        "Target frame IDs do not match",
    )

    k = np.asarray(info["k"], dtype=float).reshape(3, 3)
    d = np.asarray(info["d"], dtype=float)
    require(np.isfinite(k).all() and np.isfinite(d).all(), "Invalid calibration")
    require(k[0, 0] > 0 and k[1, 1] > 0, "Invalid focal lengths")
    require(np.allclose(k[2], [0, 0, 1]), "Unexpected intrinsic matrix")
    require(
        np.allclose(d, 0.0, atol=1e-12),
        "Nonzero distortion is not supported; rectify images first",
    )
    require(
        info.get("binning_x", 0) <= 1
        and info.get("binning_y", 0) <= 1
        and info.get("roi", {}).get("width", 0) == 0
        and info.get("roi", {}).get("height", 0) == 0,
        "Binning/ROI is not supported in this first version",
    )

    transform = tf["transform"]
    link_rotation = quaternion_matrix(transform["rotation"])
    link_center = np.array([
        transform["translation"][axis] for axis in ("x", "y", "z")
    ], dtype=float)

    optical_tf = np.asarray(
        pose["optical_convention"]["T_camera_link_from_optical"],
        dtype=float,
    )
    require(
        optical_tf.shape == (4, 4) and np.isfinite(optical_tf).all(),
        "Invalid optical convention matrix",
    )
    require(
        np.allclose(optical_tf[3], [0, 0, 0, 1]),
        "Invalid homogeneous matrix",
    )
    optical_rotation = optical_tf[:3, :3]
    require(
        np.allclose(optical_rotation.T @ optical_rotation, np.eye(3))
        and np.isclose(np.linalg.det(optical_rotation), 1.0),
        "Optical rotation is not a proper rotation",
    )

    rotation = link_rotation @ optical_rotation
    center = link_center + link_rotation @ optical_tf[:3, 3]
    require(np.isfinite(center).all(), "Invalid camera position")
    require(center[2] > args.ground_z, "Camera must be above ground plane")

    # Use full image edges to define a fixed canvas, even when rows are masked.
    corners_uv = np.array([
        [-0.5, -0.5], [width - 0.5, -0.5],
        [width - 0.5, height - 0.5], [-0.5, height - 0.5],
    ])
    corners = pixels_to_plane(
        corners_uv, k, rotation, center, args.ground_z
    )

    resolution = args.resolution
    lower = np.floor(corners[:, :2].min(axis=0) / resolution).astype(int)
    upper = np.ceil(corners[:, :2].max(axis=0) / resolution).astype(int)
    out_width, out_height = (upper - lower).tolist()
    require(
        0 < out_width <= 4000 and 0 < out_height <= 4000
        and out_width * out_height <= 4_000_000,
        "Output is too large; use a coarser resolution or check camera pose",
    )

    xmin, ymin = lower * resolution
    xmax, ymax = upper * resolution

    # PNG row 0 is the highest Y row. Sample output pixel centers.
    xs = xmin + (np.arange(out_width) + 0.5) * resolution
    ys = ymax - (np.arange(out_height) + 0.5) * resolution
    xx, yy = np.meshgrid(xs, ys)
    points = np.column_stack((
        xx.ravel(), yy.ravel(),
        np.full(xx.size, args.ground_z),
    ))

    uv, depth = world_to_pixels(points, k, rotation, center)
    keep_rows = int(math.floor(height * (1 - args.exclude_bottom)))
    require(keep_rows >= 2, "Too few source rows remain")

    # Bilinear sampling: all four source pixels must be inside retained rows.
    u, v = uv[:, 0], uv[:, 1]
    valid = (
        np.isfinite(uv).all(axis=1) & (depth > 1e-8)
        & (u >= 0) & (u < width - 1)
        & (v >= 0) & (v < keep_rows - 1)
    )
    require(valid.any(), "No valid output pixels")

    u0 = np.floor(u[valid]).astype(int)
    v0 = np.floor(v[valid]).astype(int)
    du = (u[valid] - u0)[:, None]
    dv = (v[valid] - v0)[:, None]

    colours = (
        rgb[v0, u0] * (1 - du) * (1 - dv)
        + rgb[v0, u0 + 1] * du * (1 - dv)
        + rgb[v0 + 1, u0] * (1 - du) * dv
        + rgb[v0 + 1, u0 + 1] * du * dv
    )

    rgba = np.zeros((points.shape[0], 4), dtype=np.uint8)
    rgba[valid, :3] = np.clip(np.rint(colours), 0, 255).astype(np.uint8)
    rgba[valid, 3] = 255
    rgba = rgba.reshape(out_height, out_width, 4)
    mask = valid.reshape(out_height, out_width).astype(np.uint8) * 255

    # Internal algebra check, not independent geometric accuracy validation.
    probes = np.array([
        [k[0, 2], k[1, 2]],
        [k[0, 2] + 20, k[1, 2]],
        [k[0, 2], k[1, 2] + 20],
    ])
    probe_world = pixels_to_plane(
        probes, k, rotation, center, args.ground_z
    )
    probe_back, _ = world_to_pixels(probe_world, k, rotation, center)
    error = float(np.max(np.abs(probe_back - probes)))
    require(error < 1e-6, "Projection round-trip check failed")

    metadata = {
        "schema_version": 1,
        "map_kind": "camera_single_frame_ground_plane_projection",
        "frame_id": pose["target_frame"],
        "source_sample": str(source_sample),
        "source_stamp": pose["image_stamp"],
        "resolution_m_per_pixel": resolution,
        "width": out_width,
        "height": out_height,
        "origin_lower_left": {
            "x": float(xmin), "y": float(ymin), "yaw": 0.0
        },
        "bounds": {
            "xmin": float(xmin), "xmax": float(xmax),
            "ymin": float(ymin), "ymax": float(ymax),
        },
        "pixel_center_rule": {
            "x": "xmin + (column + 0.5) * resolution",
            "y": "ymax - (row + 0.5) * resolution",
        },
        "image_axes": "right=+X, up=+Y",
        "ground_z": args.ground_z,
        "camera_optical_center": center.tolist(),
        "full_footprint_corners": corners.tolist(),
        "principal_pixel_ground_xyz": probe_world[0].tolist(),
        "exclude_bottom_fraction": args.exclude_bottom,
        "retained_source_rows": [0, keep_rows - 1],
        "valid_pixels": int(valid.sum()),
        "valid_area_m2": float(valid.sum() * resolution**2),
        "mask_semantics": "255=sampled planar projection; 0=no valid sample",
        "optical_convention": pose["optical_convention"],
        "internal_roundtrip_max_error_px": error,
        "limitations": [
            "Assumes a horizontal ground plane; not terrain reconstruction.",
            "Raised objects are displaced by planar projection.",
            "Bottom exclusion is manual, not automatic object detection.",
            "Valid mask does not certify ground or navigability.",
            "Model-based axes have not been independently calibrated.",
        ],
    }


    return rgba, mask, metadata
