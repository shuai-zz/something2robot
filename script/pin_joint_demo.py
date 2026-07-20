#!/usr/bin/env python3
"""Standalone demo: add pin / bearing joint geometry to already-split STL files.

This script takes the output of a ``--connector-mode magnet`` run (flat
interface, revolute joint) and adds cylindrical geometry for:

  pin     – simple through‑hole on both parts for a metal pin / filament rod
  bearing – bearing pocket (blind hole for the outer race) + coaxial through‑hole
            for the pin / bolt

Usage (from project root)::

    # Pin joint (3 mm filament as pin)
    uv run python script/pin_joint_demo.py                             \
        --parts-mm  result/maneki_neko_flat_v2/parts_mm                \
        --robot-result                                                 \
          result/maneki_neko_flat_v2/.../result_round1/robot_result.pkl \
        --mode pin --pin-diameter 3 --out-dir result/maneki_neko_pin

    # Bearing joint (MR106: OD 10, ID 6, thickness 3)
    uv run python script/pin_joint_demo.py                             \
        --parts-mm  result/maneki_neko_flat_v2/parts_mm                \
        --robot-result                                                 \
          result/maneki_neko_flat_v2/.../result_round1/robot_result.pkl \
        --mode bearing --bearing-od 10 --bearing-id 6                   \
        --bearing-thickness 3 --pin-diameter 6                          \
        --out-dir result/maneki_neko_bearing

The script converts joint parameters (stored in *centimetres* inside the pkl)
to millimetres internally so all CLI arguments are in mm.
"""

import argparse
import json
import os
import pickle
import shutil
import sys

import numpy as np
import trimesh


# ---------------------------------------------------------------------------
# Pickle compatibility – the robot_result.pkl references classes from the
# main module and auto_design.  We need to make them importable.
# ---------------------------------------------------------------------------
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "auto_design"))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "auto_design", "modules"))


class AutoDesignArgs:
    """Minimal compatibility shim for unpickling robot_result.pkl."""


# Make sure the pickle can find this class under both possible names.
for _mod_name in ("__main__", "run"):
    if _mod_name in sys.modules:
        sys.modules[_mod_name].AutoDesignArgs = AutoDesignArgs


def _mm(value_cm: float) -> float:
    """Convert a centimetre value (pkl convention) to millimetres."""
    return value_cm * 10.0


# ---------------------------------------------------------------------------
# Core geometry helpers
# ---------------------------------------------------------------------------

def _cylinder_mesh(center, direction, radius, height, sections=64):
    """Create a trimesh cylinder.

    Parameters
    ----------
    center : (3,) float
        Centre of the cylinder in 3D.
    direction : (3,) float
        Axis direction (will be normalised).
    radius : float
    height : float
        Total height – the cylinder will extend ``height/2`` in each direction.
    sections : int
        Circular resolution.
    """
    direction = np.asarray(direction, dtype=float)
    direction /= np.linalg.norm(direction)
    cyl = trimesh.creation.cylinder(radius=radius, height=height, sections=sections)
    # Align default Z axis → direction
    rot = trimesh.geometry.align_vectors([0, 0, 1], direction)
    cyl.apply_transform(rot)
    cyl.apply_translation(center)
    return cyl


def _subtract_cylinder(mesh, center, direction, radius, height, label=""):
    """Boolean‑subtract a cylinder from *mesh* (in‑place copy)."""
    cutter = _cylinder_mesh(center, direction, radius, height)
    result = trimesh.boolean.difference([mesh, cutter])
    if isinstance(result, trimesh.Scene):
        result = result.to_geometry()
    if not isinstance(result, trimesh.Trimesh) or len(result.faces) == 0:
        raise RuntimeError(f"Boolean difference returned empty/invalid mesh ({label})")
    return result


# ---------------------------------------------------------------------------
# Joint processing
# ---------------------------------------------------------------------------

def _process_joints(parts_mm_dir, robot_result_pkl, out_dir, mode, *,
                    pin_diameter, bearing_od, bearing_id, bearing_thickness):
    os.makedirs(out_dir, exist_ok=True)
    report = {"mode": mode, "joints": []}

    robot_result = pickle.load(open(robot_result_pkl, "rb"))
    motor_results = np.asarray(robot_result.motor_results, dtype=float)
    father_dict = robot_result.father_link_dict

    # Collect joint metadata from the link tree.
    # Deduplicate by (father, child) — a 2‑DOF joint produces two motor
    # entries for the same link pair, but we only drill one pin hole.
    queue = [robot_result.link_tree]
    joint_meta = []  # (father_name, child_name, joint_center_mm, axis_dir)
    seen_pairs = set()
    motor_idx = 0
    while queue:
        node = queue.pop(0)
        queue.extend(node.children)
        link = node.val
        if link.axis is None or np.linalg.norm(link.axis[1]) == 0:
            continue
        for axis_idx, axis_vec in enumerate(link.axis[1:]):
            m = motor_results[motor_idx]
            center_cm = (m[:3] + m[3:6]) / 2.0
            axis_raw = np.asarray(axis_vec, dtype=float)
            axis_raw /= np.linalg.norm(axis_raw)
            motor_idx += 1

            pair = (father_dict[link.name], link.name)
            if pair in seen_pairs:
                continue
            seen_pairs.add(pair)

            joint_meta.append((
                pair[0],
                pair[1],
                np.array([_mm(v) for v in center_cm]),
                axis_raw,
            ))

    # Copy original STLs → out_dir (we modify in place there)
    for fname in os.listdir(parts_mm_dir):
        if fname.lower().endswith(".stl"):
            shutil.copy2(os.path.join(parts_mm_dir, fname), os.path.join(out_dir, fname))

    for father_name, child_name, center, axis in joint_meta:
        print(f"\nProcessing joint: {father_name} ↔ {child_name}")
        print(f"  center (mm): {center}")
        print(f"  axis:        {axis}")

        parts = {}
        for name in (father_name, child_name):
            path = os.path.join(out_dir, name + ".stl")
            if not os.path.exists(path):
                print(f"  WARNING: {path} not found, skipping")
                continue
            parts[name] = trimesh.load(path, force='mesh')

        if len(parts) < 2:
            continue

        joint_entry = {
            "father": father_name,
            "child": child_name,
            "center_mm": center.tolist(),
            "axis": axis.tolist(),
        }

        if mode == "pin":
            hole_r = pin_diameter / 2.0
            hole_height = 200.0
            joint_entry["pin_diameter_mm"] = pin_diameter

            for name in (father_name, child_name):
                try:
                    mesh = parts[name]
                    mesh = _subtract_cylinder(mesh, center, axis, hole_r, hole_height,
                                              label=f"{name} pin hole")
                    mesh.export(os.path.join(out_dir, name + ".stl"))
                    parts[name] = mesh
                    print(f"  {name}: drilled Ø{pin_diameter}mm through‑hole")
                except Exception as e:
                    print(f"  {name}: FAILED — {e}")

        elif mode == "bearing":
            pocket_r = bearing_od / 2.0 + 0.1
            pocket_depth = bearing_thickness
            pin_r = pin_diameter / 2.0 + 0.2
            pin_hole_height = 200.0

            joint_entry["bearing_od_mm"] = bearing_od
            joint_entry["bearing_id_mm"] = bearing_id
            joint_entry["bearing_thickness_mm"] = bearing_thickness
            joint_entry["pin_diameter_mm"] = pin_diameter

            for name in (father_name, child_name):
                try:
                    mesh = parts[name]
                    centroid = mesh.centroid
                    to_center = center - centroid
                    outward = axis if np.dot(to_center, axis) > 0 else -axis
                    pocket_center = center + outward * (pocket_depth / 2.0)

                    mesh = _subtract_cylinder(
                        mesh, pocket_center, outward,
                        pocket_r, pocket_depth + 0.5,
                        label=f"{name} bearing pocket",
                    )
                    mesh = _subtract_cylinder(
                        mesh, center, axis,
                        pin_r, pin_hole_height,
                        label=f"{name} pin hole",
                    )
                    mesh.export(os.path.join(out_dir, name + ".stl"))
                    parts[name] = mesh
                    print(f"  {name}: bearing pocket Ø{bearing_od}×{pocket_depth}mm "
                          f"+ pin hole Ø{pin_diameter}mm")
                except Exception as e:
                    print(f"  {name}: FAILED — {e}")

        report["joints"].append(joint_entry)

    # Write report
    report_path = os.path.join(out_dir, "pin_joint_report.json")
    with open(report_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2)
    print(f"\nDone.  Report → {report_path}")
    print(f"Modified STLs → {out_dir}/")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Add pin / bearing joint geometry to split STL parts"
    )
    parser.add_argument("--parts-mm", required=True,
                        help="Path to parts_mm/ folder (mm‑scale STLs)")
    parser.add_argument("--robot-result", required=True,
                        help="Path to robot_result.pkl")
    parser.add_argument("--out-dir", required=True,
                        help="Output directory for modified STLs")

    parser.add_argument("--mode", choices=("pin", "bearing"), required=True,
                        help="Joint type")

    parser.add_argument("--pin-diameter", type=float, default=3.0,
                        help="Pin / bolt diameter in mm (default: 3.0)")
    parser.add_argument("--bearing-od", type=float, default=10.0,
                        help="Bearing outer diameter in mm (default: 10.0)")
    parser.add_argument("--bearing-id", type=float, default=6.0,
                        help="Bearing inner diameter in mm (default: 6.0)")
    parser.add_argument("--bearing-thickness", type=float, default=3.0,
                        help="Bearing thickness in mm (default: 3.0)")

    args = parser.parse_args()

    if args.mode == "bearing":
        if args.bearing_od <= args.bearing_id:
            parser.error("bearing-od must be > bearing-id")
        if args.pin_diameter >= args.bearing_id:
            parser.error("pin-diameter must be < bearing-id")

    try:
        import manifold3d  # noqa: F401
    except ImportError:
        print("WARNING: manifold3d not installed.  Install with:  pip install manifold3d")
        print("Trimesh boolean operations may fail without it.")
        print()

    _process_joints(
        parts_mm_dir=args.parts_mm,
        robot_result_pkl=args.robot_result,
        out_dir=args.out_dir,
        mode=args.mode,
        pin_diameter=args.pin_diameter,
        bearing_od=args.bearing_od,
        bearing_id=args.bearing_id,
        bearing_thickness=args.bearing_thickness,
    )


if __name__ == "__main__":
    main()
