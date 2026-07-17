import argparse
import json
import os
import pickle
import shutil
import sys

import numpy as np
import trimesh


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "auto_design"))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "auto_design", "modules"))


class AutoDesignArgs:
    """Compatibility shim for robot_result.pkl files created by run.py."""


def rotation_from_z(direction):
    direction = np.asarray(direction, dtype=float)
    direction /= np.linalg.norm(direction)
    return trimesh.geometry.align_vectors([0, 0, 1], direction)


def subtract_pocket(mesh, annotated_opening, shared_opening, axis, radius, depth):
    vertices = np.asarray(mesh.vertices)
    nearest = vertices[np.argmin(np.linalg.norm(vertices - annotated_opening, axis=1))]
    delta = nearest - shared_opening
    axial = float(np.dot(delta, axis))
    radial = float(np.linalg.norm(delta - axial * axis))
    opening = shared_opening if abs(axial) <= depth + 0.5 and radial <= radius else nearest

    inward_sign = np.sign(np.dot(nearest - opening, axis))
    if inward_sign == 0:
        inward_sign = np.sign(np.dot(mesh.centroid - opening, axis)) or 1.0
    inward = axis * inward_sign

    # Extend 0.1 mm outside the surface so the Boolean has an unambiguous cut.
    overlap = 0.1
    cutter = trimesh.creation.cylinder(radius=radius, height=depth + overlap, sections=64)
    transform = rotation_from_z(inward)
    transform[:3, 3] = opening + inward * ((depth - overlap) / 2.0)
    cutter.apply_transform(transform)

    before_volume = abs(float(mesh.volume))
    result = trimesh.boolean.difference([mesh, cutter], engine="blender", check_volume=False)
    if isinstance(result, trimesh.Scene):
        result = result.to_geometry()
    if not isinstance(result, trimesh.Trimesh) or len(result.faces) == 0:
        raise RuntimeError("Boolean difference returned an empty or invalid mesh")

    components = result.split(only_watertight=False)
    component_faces = sorted((len(component.faces) for component in components), reverse=True)
    cleaned_small_fragments = False
    if len(components) > 1 and component_faces[1] < component_faces[0] * 0.01:
        result = max(components, key=lambda component: len(component.faces))
        cleaned_small_fragments = True

    return result, {
        "annotated_opening_mm": annotated_opening.tolist(),
        "actual_opening_mm": opening.tolist(),
        "inward_axis": inward.tolist(),
        "removed_volume_mm3": before_volume - abs(float(result.volume)),
        "components_before_cleanup": len(components),
        "cleaned_small_fragments": cleaned_small_fragments,
    }


def main():
    parser = argparse.ArgumentParser(description="Cut precise magnet pockets into exported mm-scale STL parts.")
    parser.add_argument("--parts-mm", required=True)
    parser.add_argument("--robot-result", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--diameter", type=float, default=6.0)
    parser.add_argument("--thickness", type=float, default=2.0)
    parser.add_argument("--clearance", type=float, default=0.2)
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    for filename in os.listdir(args.parts_mm):
        if filename.lower().endswith(".stl"):
            shutil.copy2(os.path.join(args.parts_mm, filename), os.path.join(args.out_dir, filename))

    robot_result = pickle.load(open(args.robot_result, "rb"))
    radius = (args.diameter + args.clearance) / 2.0
    depth = args.thickness + args.clearance / 2.0
    report = {"diameter_mm": radius * 2, "depth_mm": depth, "engine": "blender", "pockets": []}

    queue = [robot_result.link_tree]
    result_idx = 0
    while queue:
        node = queue.pop(0)
        queue.extend(node.children)
        link = node.val
        if link.axis is None or np.linalg.norm(link.axis[1]) == 0:
            continue
        for _ in link.axis[1:]:
            motor_result = np.asarray(robot_result.motor_results[result_idx], dtype=float)
            annotated_opening = (motor_result[:3] + motor_result[3:6]) * 5.0
            axis = motor_result[3:6] - motor_result[:3]
            axis /= np.linalg.norm(axis)
            father_name = robot_result.father_link_dict[link.name]
            part_names = (father_name, link.name)
            meshes = {
                name: trimesh.load_mesh(os.path.join(args.out_dir, name + ".stl"), process=False)
                for name in part_names
            }
            nearest = [
                np.asarray(mesh.vertices)[np.argmin(np.linalg.norm(np.asarray(mesh.vertices) - annotated_opening, axis=1))]
                for mesh in meshes.values()
            ]
            shared_opening = np.mean(nearest, axis=0)

            for part_name, mesh in meshes.items():
                result, details = subtract_pocket(
                    mesh, annotated_opening, shared_opening, axis, radius, depth
                )
                result.export(os.path.join(args.out_dir, part_name + ".stl"))
                details.update({
                    "joint_link": link.name,
                    "part": part_name,
                    "watertight": bool(result.is_watertight),
                    "components": len(result.split(only_watertight=False)),
                    "faces": len(result.faces),
                })
                report["pockets"].append(details)
            result_idx += 1

    with open(os.path.join(args.out_dir, "boolean_report.json"), "w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
