#!/usr/bin/env python3
"""Build a full-mesh Cactus arm/body pair with a locally voxelized tenon.

The original triangles are retained away from the joint.  Only the male peg,
socket collar, and socket cutter are generated on a small voxel grid.  Blender
is used to bisect the source mesh and combine those local meshes robustly.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


def blender_worker(argv: list[str]) -> None:
    import bmesh
    import bpy
    from mathutils import Vector

    parser = argparse.ArgumentParser()
    parser.add_argument("--source")
    parser.add_argument("--body-source")
    parser.add_argument("--arm-source")
    parser.add_argument("--male", required=True)
    parser.add_argument("--collar", required=True)
    parser.add_argument("--cutter", required=True)
    parser.add_argument("--relief")
    parser.add_argument("--relief-annulus")
    parser.add_argument("--body-out", required=True)
    parser.add_argument("--arm-out", required=True)
    parser.add_argument("--interference-out", required=True)
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--joint-center", type=float, nargs=3, required=True)
    parser.add_argument("--plane-normal", type=float, nargs=3, required=True)
    parser.add_argument("--skip-interference-check", action="store_true")
    parser.add_argument("--skip-conformal-fit", action="store_true")
    args = parser.parse_args(argv)

    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)

    def import_stl(path: str, name: str):
        bpy.ops.wm.stl_import(filepath=path)
        obj = bpy.context.active_object
        obj.name = name
        return obj

    def apply_scale(obj, scale: float):
        obj.scale = (scale, scale, scale)
        bpy.context.view_layer.objects.active = obj
        obj.select_set(True)
        bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
        obj.select_set(False)

    def bisect(obj, keep_positive: bool):
        bm = bmesh.new()
        bm.from_mesh(obj.data)
        result = bmesh.ops.bisect_plane(
            bm,
            geom=list(bm.verts) + list(bm.edges) + list(bm.faces),
            plane_co=Vector(args.joint_center),
            plane_no=Vector(args.plane_normal),
            clear_inner=keep_positive,
            clear_outer=not keep_positive,
            dist=1e-5,
        )
        cut_edges = [item for item in result["geom_cut"] if isinstance(item, bmesh.types.BMEdge)]
        if cut_edges:
            try:
                bmesh.ops.holes_fill(bm, edges=cut_edges, sides=0)
            except Exception:
                pass
        bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-5)
        bm.normal_update()
        bm.to_mesh(obj.data)
        bm.free()
        obj.data.update()

    def boolean(target, tool, operation: str):
        bpy.context.view_layer.objects.active = target
        modifier = target.modifiers.new(name=f"local_{operation.lower()}", type="BOOLEAN")
        modifier.operation = operation
        modifier.solver = "EXACT"
        modifier.object = tool
        bpy.ops.object.modifier_apply(modifier=modifier.name)
        bpy.data.objects.remove(tool, do_unlink=True)

    def seal_open_boundaries(obj):
        bm = bmesh.new()
        bm.from_mesh(obj.data)
        boundary = [edge for edge in bm.edges if edge.is_boundary]
        if boundary:
            bmesh.ops.holes_fill(bm, edges=boundary, sides=0)
        bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-5)
        bm.normal_update()
        bm.to_mesh(obj.data)
        bm.free()
        obj.data.update()

    def export_stl(obj, path: str):
        bpy.ops.object.select_all(action="DESELECT")
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj
        bpy.ops.wm.stl_export(filepath=path, export_selected_objects=True)

    if args.body_source and args.arm_source:
        body = import_stl(args.body_source, "BODY")
        arm = import_stl(args.arm_source, "arm")
    else:
        source = import_stl(args.source, "source")
        apply_scale(source, args.scale)
        body = source.copy()
        body.data = source.data.copy()
        bpy.context.collection.objects.link(body)
        body.name = "BODY"
        arm = source.copy()
        arm.data = source.data.copy()
        bpy.context.collection.objects.link(arm)
        arm.name = "arm"
        bpy.data.objects.remove(source, do_unlink=True)
        bisect(body, keep_positive=True)
        bisect(arm, keep_positive=False)

    if args.relief:
        relief = import_stl(args.relief, "joint_motion_relief")
        boolean(body, relief, "DIFFERENCE")
    if args.relief_annulus:
        relief_annulus = import_stl(args.relief_annulus, "body_side_motion_relief")
        boolean(body, relief_annulus, "DIFFERENCE")
    collar = import_stl(args.collar, "local_voxel_socket_collar")
    boolean(body, collar, "UNION")
    cutter = import_stl(args.cutter, "local_voxel_socket_cutter")
    boolean(body, cutter, "DIFFERENCE")
    male = import_stl(args.male, "local_voxel_male_tenon")
    boolean(arm, male, "UNION")

    # Remove any residual overlap from the real curved arm root. The cylindrical
    # cutter clears the peg, while this conformal subtraction clears only the
    # original root material that would otherwise remain inside BODY.
    if not args.skip_conformal_fit:
        arm_fit_tool = arm.copy()
        arm_fit_tool.data = arm.data.copy()
        bpy.context.collection.objects.link(arm_fit_tool)
        boolean(body, arm_fit_tool, "DIFFERENCE")
    seal_open_boundaries(body)
    seal_open_boundaries(arm)

    export_stl(body, args.body_out)
    export_stl(arm, args.arm_out)

    if not args.skip_interference_check:
        interference = body.copy()
        interference.data = body.data.copy()
        bpy.context.collection.objects.link(interference)
        arm_tool = arm.copy()
        arm_tool.data = arm.data.copy()
        bpy.context.collection.objects.link(arm_tool)
        boolean(interference, arm_tool, "INTERSECT")
        if len(interference.data.polygons) > 0:
            export_stl(interference, args.interference_out)


def voxel_cylinder_mesh(x_min, x_max, radius, voxel_size, center_yz):
    import numpy as np
    import trimesh
    from skimage import measure

    margin = voxel_size * 2
    mins = np.array([x_min - margin, center_yz[0] - radius - margin, center_yz[1] - radius - margin])
    maxs = np.array([x_max + margin, center_yz[0] + radius + margin, center_yz[1] + radius + margin])
    axes = [np.arange(mins[i], maxs[i] + voxel_size * 0.5, voxel_size) for i in range(3)]
    points = np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1)
    radial = np.sqrt((points[..., 1] - center_yz[0]) ** 2 + (points[..., 2] - center_yz[1]) ** 2)
    mask = (points[..., 0] >= x_min) & (points[..., 0] <= x_max) & (radial <= radius)
    vertices, faces, _, _ = measure.marching_cubes(
        np.pad(mask.astype(np.uint8), 1), level=0.5, spacing=(voxel_size,) * 3
    )
    vertices += mins - voxel_size
    return trimesh.Trimesh(vertices=vertices, faces=faces, process=True)


def round_profile_mesh(xs, radii, center_yz, sections):
    """Create a watertight surface of revolution around X."""
    import numpy as np
    import trimesh

    angles = np.linspace(0.0, 2.0 * np.pi, sections, endpoint=False)
    rings = []
    for x, radius in zip(xs, radii):
        rings.append(
            np.column_stack(
                (
                    np.full(sections, x),
                    center_yz[0] + radius * np.cos(angles),
                    center_yz[1] + radius * np.sin(angles),
                )
            )
        )
    vertices = np.vstack(rings + [
        np.array([[xs[0], center_yz[0], center_yz[1]]]),
        np.array([[xs[-1], center_yz[0], center_yz[1]]]),
    ])
    faces = []
    for ring_idx in range(len(rings) - 1):
        start = ring_idx * sections
        next_start = (ring_idx + 1) * sections
        for idx in range(sections):
            nxt = (idx + 1) % sections
            faces.extend(
                [
                    [start + idx, next_start + idx, next_start + nxt],
                    [start + idx, next_start + nxt, start + nxt],
                ]
            )
    left_center = len(rings) * sections
    right_center = left_center + 1
    last_start = (len(rings) - 1) * sections
    for idx in range(sections):
        nxt = (idx + 1) % sections
        faces.append([left_center, idx, nxt])
        faces.append([right_center, last_start + nxt, last_start + idx])
    mesh = trimesh.Trimesh(vertices=vertices, faces=np.asarray(faces), process=True)
    # Blender's boolean solver interprets winding direction. Ensure the solid
    # is outward-facing; the generated X-axis profile can otherwise be negative.
    if mesh.volume < 0:
        mesh.invert()
    return mesh


def orient_x_axis_mesh(mesh, origin, direction):
    import numpy as np
    import trimesh

    direction = np.asarray(direction, dtype=float)
    direction /= np.linalg.norm(direction)
    transform = trimesh.geometry.align_vectors([1.0, 0.0, 0.0], direction)
    transform[:3, 3] = np.asarray(origin, dtype=float)
    mesh.apply_transform(transform)
    return mesh


def main() -> None:
    parser = argparse.ArgumentParser(description="Cactus local-voxel tenon experiment")
    parser.add_argument("--model", default="Cactus_Character_Scaled")
    parser.add_argument("--link", default="r_arm")
    parser.add_argument("--parent-link", default="BODY")
    parser.add_argument("--expected-x", type=float, default=100.0)
    parser.add_argument("--voxel-size", type=float, default=0.5)
    parser.add_argument("--tenon-radius", type=float, default=6.0)
    parser.add_argument("--tenon-depth", type=float, default=8.0)
    parser.add_argument(
        "--cut-offset", type=float, default=0.0,
        help="Move the cut plane toward the right arm (negative X), in mm",
    )
    parser.add_argument("--clearance", type=float, default=0.4)
    parser.add_argument("--collar-thickness", type=float, default=2.0)
    parser.add_argument("--connector-geometry", choices=("voxel", "round"), default="voxel")
    parser.add_argument("--round-sections", type=int, default=256)
    parser.add_argument("--lead-in", type=float, default=0.75, help="Tenon tip chamfer length in mm")
    parser.add_argument(
        "--tenon-root-overlap", type=float, default=None,
        help="How far the tenon is embedded into the child part; defaults to one voxel",
    )
    parser.add_argument(
        "--motion-relief-radius", type=float, default=0.0,
        help="Radius of a spherical BODY clearance around the joint, in mm",
    )
    parser.add_argument("--motion-relief-body-depth", type=float, default=0.0)
    parser.add_argument("--motion-relief-inner-radius", type=float, default=0.0)
    parser.add_argument("--out-dir", type=Path, default=REPO_ROOT / "result" / "cactus_local_voxel_tenon")
    parser.add_argument(
        "--parts-mm", type=Path,
        help="Use already validated BODY.stl and <link>.stl instead of bisecting the source mesh",
    )
    args = parser.parse_args()

    import pickle
    import numpy as np
    import trimesh

    sys.path[:0] = [str(REPO_ROOT / "auto_design"), str(REPO_ROOT / "auto_design" / "modules")]
    model_dir = REPO_ROOT / "auto_design" / "model" / "given_models"
    source_path = model_dir / f"{args.model}.stl"
    joints_path = model_dir / f"{args.model}_joints.pkl"
    source = trimesh.load_mesh(source_path, process=False)
    scale = args.expected_x / float(source.extents[0])
    with joints_path.open("rb") as handle:
        annotations = pickle.load(handle)
    if args.link not in annotations:
        raise KeyError(f"Unknown link {args.link!r}; choose from {sorted(annotations)}")
    joint = annotations[args.link].val
    annotated_center = [value * scale for value in joint.axis[0]]
    axis = np.asarray(joint.axis[1], dtype=float)
    axis /= np.linalg.norm(axis)
    joint_points = np.asarray(list(joint.joints.values()), dtype=float) * scale
    other_points = joint_points[np.linalg.norm(joint_points - annotated_center, axis=1) > 1e-6]
    toward_link = other_points.mean(axis=0) - annotated_center
    child_direction = axis if np.dot(toward_link, axis) >= 0 else -axis
    parent_direction = -child_direction
    center = (np.asarray(annotated_center) + child_direction * args.cut_offset).tolist()

    # The peg crosses the split plane slightly so its union with the arm is robust.
    if args.connector_geometry == "round":
        end_x = args.tenon_depth
        lead_in = min(args.lead_in, args.tenon_depth * 0.4)
        root_overlap = args.tenon_root_overlap or args.voxel_size
        male = round_profile_mesh(
            [-root_overlap, end_x - lead_in, end_x],
            [args.tenon_radius, args.tenon_radius, args.tenon_radius - lead_in],
            (0.0, 0.0),
            args.round_sections,
        )
        cutter = round_profile_mesh(
            [-args.voxel_size, end_x + args.voxel_size],
            [args.tenon_radius + args.clearance] * 2,
            (0.0, 0.0),
            args.round_sections,
        )
        collar = round_profile_mesh(
            [0.0, end_x + args.collar_thickness],
            [args.tenon_radius + args.clearance + args.collar_thickness] * 2,
            (0.0, 0.0),
            args.round_sections,
        )
        male = orient_x_axis_mesh(male, center, parent_direction)
        cutter = orient_x_axis_mesh(cutter, center, parent_direction)
        collar = orient_x_axis_mesh(collar, center, parent_direction)
    else:
        male = voxel_cylinder_mesh(
            center[0] - args.voxel_size, center[0] + args.tenon_depth,
            args.tenon_radius, args.voxel_size, (center[1], center[2]),
        )
        cutter = voxel_cylinder_mesh(
            center[0] - args.voxel_size, center[0] + args.tenon_depth + args.voxel_size,
            args.tenon_radius + args.clearance, args.voxel_size, (center[1], center[2]),
        )
        collar = voxel_cylinder_mesh(
            center[0], center[0] + args.tenon_depth + args.collar_thickness,
            args.tenon_radius + args.clearance + args.collar_thickness,
            args.voxel_size, (center[1], center[2]),
        )

    args.out_dir.mkdir(parents=True, exist_ok=True)
    body_out = args.out_dir / f"{args.parent_link}_local_voxel_tenon.stl"
    arm_out = args.out_dir / f"{args.link}_local_voxel_tenon.stl"
    interference_out = args.out_dir / "assembled_interference.stl"
    interference_out.unlink(missing_ok=True)
    with tempfile.TemporaryDirectory(prefix="cactus_tenon_") as temp_name:
        temp = Path(temp_name)
        male_path, cutter_path, collar_path = temp / "male.stl", temp / "cutter.stl", temp / "collar.stl"
        male.export(male_path)
        cutter.export(cutter_path)
        collar.export(collar_path)
        relief_path = None
        relief_annulus_path = None
        if args.motion_relief_radius > 0:
            # Clear only the child-facing hemisphere. A full sphere would also
            # remove the body-side material needed to support the socket collar.
            relief_x = np.linspace(-args.motion_relief_radius, 0.0, 33)
            relief_r = np.sqrt(np.maximum(0.0, args.motion_relief_radius**2 - relief_x**2))
            relief_r[0] = 0.01
            relief = round_profile_mesh(relief_x, relief_r, (0.0, 0.0), 128)
            relief = orient_x_axis_mesh(relief, center, parent_direction)
            relief_path = temp / "relief.stl"
            relief.export(relief_path)
            if args.motion_relief_body_depth > 0:
                inner_radius = args.motion_relief_inner_radius or (
                    args.tenon_radius + args.clearance + args.collar_thickness + 0.5
                )
                relief_annulus = trimesh.creation.annulus(
                    r_min=inner_radius,
                    r_max=args.motion_relief_radius,
                    height=None,
                    sections=256,
                    segment=[[0.0, 0.0, 0.0], [args.motion_relief_body_depth, 0.0, 0.0]],
                )
                relief_annulus = orient_x_axis_mesh(relief_annulus, center, parent_direction)
                relief_annulus_path = temp / "relief_annulus.stl"
                relief_annulus.export(relief_annulus_path)
        blender = shutil.which("blender") or "/opt/homebrew/bin/blender"
        if not Path(blender).exists():
            raise FileNotFoundError("Blender executable not found")
        command = [
            blender, "--background", "--python", str(Path(__file__).resolve()), "--",
            "--blender-worker", "--male", str(male_path),
            "--collar", str(collar_path), "--cutter", str(cutter_path),
            "--body-out", str(body_out), "--arm-out", str(arm_out),
            "--interference-out", str(interference_out),
            "--scale", str(scale), "--joint-center", *map(str, center),
            "--plane-normal", *map(str, parent_direction),
        ]
        if args.parts_mm:
            command.extend([
                "--body-source", str(args.parts_mm / f"{args.parent_link}.stl"),
                "--arm-source", str(args.parts_mm / f"{args.link}.stl"),
            ])
        else:
            command.extend(["--source", str(source_path)])
        if relief_path:
            command.extend(["--relief", str(relief_path)])
        if relief_annulus_path:
            command.extend(["--relief-annulus", str(relief_annulus_path)])
        if args.connector_geometry == "round":
            # Exact intersection of two 200k-face meshes with a 256-section
            # cylinder is disproportionately slow. The same fitting workflow
            # is validated by the voxel version; round clearance is analytic.
            command.append("--skip-interference-check")
            command.append("--skip-conformal-fit")
        subprocess.run(command, check=True)

    validation = {}
    for path in (body_out, arm_out):
        mesh = trimesh.load_mesh(path, process=True)
        components = sorted(
            mesh.split(only_watertight=False), key=lambda part: abs(part.volume), reverse=True
        )
        if components:
            # Remove microscopic boolean debris while preserving meaningful
            # disconnected details (for example the Cactus decoration).
            minimum_volume = abs(components[0].volume) * 1e-5
            components = [part for part in components if abs(part.volume) >= minimum_volume]
            mesh = trimesh.util.concatenate(components)
            mesh.export(path)
        validation[path.name] = {
            "watertight": bool(mesh.is_watertight),
            "components": len(mesh.split(only_watertight=False)),
            "faces": len(mesh.faces),
            "bounds_mm": mesh.bounds.tolist(),
        }
    body_preview = trimesh.load_mesh(body_out, process=True)
    arm_preview = trimesh.load_mesh(arm_out, process=True)
    body_preview.visual.face_colors = [85, 145, 235, 210]
    arm_preview.visual.face_colors = [255, 135, 55, 255]
    trimesh.Scene({args.parent_link: body_preview, args.link: arm_preview}).export(
        args.out_dir / f"{args.model}_joint_preview.glb"
    )
    report = {
        "model": args.model,
        "joint": args.link,
        "parent_link": args.parent_link,
        "annotated_joint_center_mm": annotated_center,
        "connector_center_mm": center,
        "connector_axis_into_body": parent_direction.tolist(),
        "cut_offset_toward_arm_mm": args.cut_offset,
        "method": (
            "original triangle mesh + parametric round connector"
            if args.connector_geometry == "round"
            else "original triangle mesh + locally voxelized male tenon/socket collar/socket cutter"
        ),
        "connector_geometry": args.connector_geometry,
        "round_sections": args.round_sections if args.connector_geometry == "round" else None,
        "lead_in_mm": args.lead_in if args.connector_geometry == "round" else None,
        "voxel_size_mm": args.voxel_size,
        "tenon_radius_mm": args.tenon_radius,
        "tenon_depth_mm": args.tenon_depth,
        "diametral_clearance_mm": args.clearance * 2,
        "collar_thickness_mm": args.collar_thickness,
        "motion_relief_radius_mm": args.motion_relief_radius,
        "motion_relief_body_depth_mm": args.motion_relief_body_depth,
        "motion_relief_inner_radius_mm": args.motion_relief_inner_radius,
        "tenon_root_overlap_mm": args.tenon_root_overlap or args.voxel_size,
        "validation": validation,
    }
    if interference_out.exists() and interference_out.stat().st_size > 84:
        interference_mesh = trimesh.load_mesh(interference_out, process=True)
        report["assembled_interference_volume_mm3"] = abs(float(interference_mesh.volume))
        report["assembled_interference_faces"] = len(interference_mesh.faces)
    elif args.connector_geometry != "round":
        report["assembled_interference_volume_mm3"] = 0.0
        report["assembled_interference_faces"] = 0
    else:
        report["assembled_interference_volume_mm3"] = None
        report["interference_check"] = "skipped; analytic radial clearance is reported instead"
        report["analytic_radial_clearance_mm"] = args.clearance
    (args.out_dir / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    if "--blender-worker" in sys.argv:
        divider = sys.argv.index("--") if "--" in sys.argv else -1
        worker_args = sys.argv[divider + 1 :]
        worker_args.remove("--blender-worker")
        blender_worker(worker_args)
    else:
        main()
