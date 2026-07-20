#!/usr/bin/env python3
"""Approximate a one-axis joint motion sweep using signed-distance queries."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
import open3d as o3d
import trimesh


def rotation_matrix(axis: np.ndarray, angle_degrees: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    angle = np.deg2rad(angle_degrees)
    cross = np.array(
        [[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]],
        dtype=float,
    )
    return np.eye(3) * np.cos(angle) + (1 - np.cos(angle)) * np.outer(axis, axis) + np.sin(angle) * cross


def ray_scene(mesh: trimesh.Trimesh) -> o3d.t.geometry.RaycastingScene:
    legacy = o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector(mesh.vertices),
        o3d.utility.Vector3iVector(mesh.faces),
    )
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(legacy))
    return scene


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--body", type=Path, required=True)
    parser.add_argument("--moving", type=Path, required=True)
    parser.add_argument("--center", type=float, nargs=3, required=True)
    parser.add_argument("--axis", type=float, nargs=3, required=True)
    parser.add_argument("--angle-min", type=int, default=-90)
    parser.add_argument("--angle-max", type=int, default=90)
    parser.add_argument("--angle-step", type=int, default=10)
    parser.add_argument("--samples", type=int, default=100000)
    parser.add_argument("--penetration-tolerance", type=float, default=0.2)
    parser.add_argument(
        "--minimum-collision-samples", type=int, default=5,
        help="Reject isolated signed-distance outliers below this count",
    )
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()

    body = trimesh.load_mesh(args.body, process=True)
    moving = trimesh.load_mesh(args.moving, process=True)
    if not body.is_watertight or not moving.is_watertight:
        raise ValueError("Both meshes must be watertight")

    # Include all vertices for sharp/local features and uniform surface samples
    # for broad faces. Fixed RNG keeps comparisons reproducible.
    np.random.seed(42)
    surface, _ = trimesh.sample.sample_surface(moving, args.samples)
    sample_points = np.vstack((moving.vertices, surface))
    center = np.asarray(args.center, dtype=float)
    axis = np.asarray(args.axis, dtype=float)
    axis /= np.linalg.norm(axis)
    scene = ray_scene(body)

    results = []
    for angle in range(args.angle_min, args.angle_max + 1, args.angle_step):
        rotation = rotation_matrix(axis, angle)
        moved = (sample_points - center) @ rotation.T + center
        signed = scene.compute_signed_distance(
            o3d.core.Tensor(moved.astype(np.float32))
        ).numpy()
        penetrating = signed < -args.penetration_tolerance
        penetrating_count = int(np.count_nonzero(penetrating))
        collision_geometry = {}
        if penetrating_count:
            relative = moved[penetrating] - center
            axial = relative @ axis
            radial_vectors = relative - np.outer(axial, axis)
            radial = np.linalg.norm(radial_vectors, axis=1)
            distance = np.linalg.norm(relative, axis=1)
            collision_geometry = {
                "collision_axial_min_mm": float(axial.min()),
                "collision_axial_max_mm": float(axial.max()),
                "collision_radial_min_mm": float(radial.min()),
                "collision_radial_max_mm": float(radial.max()),
                "collision_distance_max_mm": float(distance.max()),
            }
        results.append(
            {
                "angle_degrees": angle,
                "collision": penetrating_count >= args.minimum_collision_samples,
                "penetrating_samples": penetrating_count,
                "penetrating_percent": float(100 * np.mean(penetrating)),
                "max_penetration_mm": float(max(0.0, -signed.min())),
                "minimum_signed_distance_mm": float(signed.min()),
                **collision_geometry,
            }
        )

    zero_index = next(i for i, row in enumerate(results) if row["angle_degrees"] == 0)
    safe_low = safe_high = 0
    for index in range(zero_index - 1, -1, -1):
        if results[index]["collision"]:
            break
        safe_low = results[index]["angle_degrees"]
    for index in range(zero_index + 1, len(results)):
        if results[index]["collision"]:
            break
        safe_high = results[index]["angle_degrees"]

    report = {
        "body": str(args.body.resolve()),
        "moving": str(args.moving.resolve()),
        "center_mm": center.tolist(),
        "axis": axis.tolist(),
        "penetration_tolerance_mm": args.penetration_tolerance,
        "minimum_collision_samples": args.minimum_collision_samples,
        "surface_sample_count": len(sample_points),
        "safe_continuous_range_around_zero_degrees": [safe_low, safe_high],
        "method": "sampled moving-part surface queried against BODY signed distance",
        "results": results,
    }
    args.out_dir.mkdir(parents=True, exist_ok=True)
    (args.out_dir / "motion_report.json").write_text(json.dumps(report, indent=2) + "\n")
    with (args.out_dir / "motion_report.csv").open("w", newline="") as handle:
        fieldnames = list(dict.fromkeys(key for row in results for key in row.keys()))
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
