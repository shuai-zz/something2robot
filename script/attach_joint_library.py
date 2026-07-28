"""Carve snap-fit peg holes and magnet holes into decomposed link STLs.

Joint plan: built-in presets (JOINT_PLANS below) or a JSON file per model —
see load_joint_plan() for resolution order and the file format. To support a
new model, add <model_stem>_joint_plan.json next to its _joints.pkl.
  neck                    -> blind magnet holes (default dia 9 x depth 4) on BODY and HEAD
  hips / knees / ankles   -> keyed peg holes (snap-fit) on BOTH parent and child

Peg hole cutter: auto_design/model/joint_models/peg_joint/7mm_keyed_peg_hole_cutter_clearance_0p20.stl
  - axis along +Z, entry at z=0, snap groove at z=2.7..4.6, straight guide to z=11.5
  - keyed anti-rotation flats are perpendicular to the cutter Y axis
  - 0.2 mm clearance is already built into the cutter, use it directly for boolean

Placement rules:
  - hole axis = joint separation direction, drilled from the cut face INTO each link
    (parent gets the hole pointing away from the child and vice versa)
  - anti-rotation flats are kept parallel to the figure's chest/back plane
    (cutter Y axis mapped onto the model Y axis as closely as the hole axis allows)
  - when one link receives two holes from opposite ends (e.g. lower legs), the
    holes are shortened so a wall remains between them (see depth overrides and
    the wall report printed at the end)

Positions: joint annotations are in source-STL units; parts_mm STLs are in
millimetres. unit_scale = expected_x_mm / source_stl_x_extent converts them.

Example:
  uv run python script/attach_joint_library.py \
    --parts-mm result/mario_links_only_v6_newanno/parts_mm \
    --joints-pkl auto_design/model/given_models/Mario_Character_Image_1020080024_scaled_joints.pkl \
    --source-stl auto_design/model/given_models/Mario_Character_Image_1020080024_scaled.stl \
    --expected-x 100 --out-dir result/mario_joints_attached
"""

import argparse
import json
import os
import sys

import numpy as np
import trimesh

JOINT_MODELS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'auto_design', 'model', 'joint_models')
PEG_CUTTER_PATH = os.path.join(JOINT_MODELS_DIR, 'peg_joint',
                               '7mm_keyed_peg_hole_cutter_clearance_0p20.stl')
PEG_FULL_DEPTH = 11.5  # cutter spans z=0 (entry) .. z=11.5 (deep end)

# Joint plans per model. depth_parent / depth_child override PEG_FULL_DEPTH.
JOINT_PLANS = {
    'mario': [
        {'joint': 'neck',    'type': 'magnet', 'parent': 'BODY',        'child': 'HEAD'},
        {'joint': 'l_hip',   'type': 'peg',    'parent': 'BODY',        'child': 'L_LEG'},
        {'joint': 'r_hip',   'type': 'peg',    'parent': 'BODY',        'child': 'R_LEG'},
        # lower legs are only ~14 mm tall: shorten both their holes to keep a wall
        {'joint': 'l_knee',  'type': 'peg',    'parent': 'L_LEG',       'child': 'L_LOWER_LEG', 'depth_child': 6.0},
        {'joint': 'r_knee',  'type': 'peg',    'parent': 'R_LEG',       'child': 'R_LOWER_LEG', 'depth_child': 6.0},
        # ankles: drill straight down (vertical) instead of along the foot so the
        # peg can be inserted without interference
        {'joint': 'l_ankle', 'type': 'peg',    'parent': 'L_LOWER_LEG', 'child': 'L_FOOT',      'depth_parent': 6.0, 'axis': [0, 0, -1]},
        {'joint': 'r_ankle', 'type': 'peg',    'parent': 'R_LOWER_LEG', 'child': 'R_FOOT',      'depth_parent': 6.0, 'axis': [0, 0, -1]},
    ],
    'cactus': [
        {'joint': 'body_waist', 'type': 'peg', 'parent': 'BODY', 'child': 'base'},
        # arms: use the shoulder's local segment direction (arm_r1 -> arm_r2),
        # not the shoulder->tip diagonal which curves up
        {'joint': 'arm_r1',     'type': 'peg', 'parent': 'base', 'child': 'r_arm', 'axis': [-1, 0, 0]},
        {'joint': 'arm_11',     'type': 'peg', 'parent': 'base', 'child': 'l_arm', 'axis': [1, 0, 0]},
    ],
}


def load_link_tree(joints_pkl):
    sys.path.insert(0, 'auto_design')
    sys.path.insert(0, 'auto_design/modules')
    import pickle
    with open(joints_pkl, 'rb') as f:
        return pickle.load(f)


def collect_link_info(tree):
    """Return {link_name: {'joints': {name: pos}, 'parent': name|None}}."""
    info = {}

    def walk(node, parent_name):
        info[node.val.name] = {
            'joints': {jn: np.asarray(jp, dtype=float) for jn, jp in node.val.joints.items()},
            'parent': parent_name,
        }
        for c in node.children:
            walk(c, node.val.name)
    walk(tree['BODY'], None)
    return info


def rotation_with_axis(target_axis, flat_normal=np.array([1.0, 0.0, 0.0])):
    """Rotation mapping lib Z onto target_axis, keeping the keyed flats
    (lib Y normal) as close to flat_normal as the axis allows.

    flat_normal defaults to model X: the anti-rotation flat planes face
    left/right, i.e. rotated 90 deg about the peg axis vs the original
    chest/back orientation (the hinge swing direction then runs across
    the flats instead of along them)."""
    z = np.asarray(target_axis, dtype=float)
    z /= np.linalg.norm(z)
    y = flat_normal - (flat_normal @ z) * z
    n = np.linalg.norm(y)
    if n < 1e-6:
        # axis parallel to flat_normal (e.g. horizontal arm holes with the
        # default X flat normal): fall back to Y, then Z, then X
        for cand in ([0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]):
            y = np.asarray(cand) - (np.asarray(cand) @ z) * z
            n = np.linalg.norm(y)
            if n > 1e-6:
                break
    y /= n
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


def peg_cutter(direction, depth, flip=False):
    """Keyed peg hole cutter placed with its entry at the origin and its
    axis along `direction`, truncated to `depth` (capped).

    The cutter's snap groove (the "big end", z=2.7..4.6 in library coords)
    sits near its z=0 end. For full-depth holes the groove must be at the
    DEEP end of the hole ("big end toward the model interior"): flip the
    cutter so the smooth guide enters first and the groove lands at
    z=6.9..8.8. Short holes (depth overrides) keep the groove within the
    shallow hole and are used unflipped.
    """
    cutter = trimesh.load(PEG_CUTTER_PATH)
    if flip:
        cutter.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0]))
        cutter.apply_translation([0, 0, PEG_FULL_DEPTH])
    if depth < PEG_FULL_DEPTH:
        cutter = cutter.slice_plane([0, 0, depth], [0, 0, -1], cap=True)
    rot = rotation_with_axis(direction)
    t = np.eye(4)
    t[:3, :3] = rot
    cutter.apply_transform(t)
    return cutter


ENTRY_RELIEF = 8.0   # mm the entry relief channel extends outward from the surface
ENTRY_RELIEF_LEN = 10.0  # relief channel length: surface-8 .. surface+2


def peg_entry_relief(direction, flip):
    """Full-width channel through the surface lip zone: the peg hole profile
    carved from ENTRY_RELIEF mm outside the surface to 2 mm inside it, so a
    sloped cut face can never leave a lip blocking the peg. Only the entry
    zone is carved (truncated before the deep snap-groove depth), the snap
    groove position is governed by the main cutter."""
    relief = trimesh.load(PEG_CUTTER_PATH)
    if flip:
        relief.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0]))
        relief.apply_translation([0, 0, PEG_FULL_DEPTH])
    relief = relief.slice_plane([0, 0, ENTRY_RELIEF_LEN], [0, 0, -1], cap=True)
    rot = rotation_with_axis(direction)
    t = np.eye(4)
    t[:3, :3] = rot
    relief.apply_transform(t)
    relief.apply_translation(-np.asarray(direction, dtype=float) * ENTRY_RELIEF)
    return relief


def drill_magnet_hole(link_mesh, joint_pos, diameter, depth, direction):
    """Drill a blind hole at the surface near joint_pos along `direction`.

    Ray-cast from outside toward the link to find the actual surface, then
    carve a cylinder of `depth` from the surface inward.
    """
    direction = np.asarray(direction, dtype=float)
    direction /= np.linalg.norm(direction)
    origin = joint_pos - direction * 30.0  # start outside, cast toward the link
    ray = trimesh.ray.ray_triangle.RayMeshIntersector(link_mesh)
    locs, _, _ = ray.intersects_location([origin], [direction])
    if len(locs) == 0:
        raise RuntimeError(f'No surface found along {direction} from {origin}')
    # first hit along the ray
    t = ((locs - origin) @ direction)
    surface = locs[np.argmin(t)]
    # blind end of the hole, `depth` inside the material
    floor = surface + direction * depth
    # cylinder spans from the floor back out through the surface (plus margin)
    height = depth + 8.0
    center = floor - direction * (height / 2.0)
    cyl = trimesh.creation.cylinder(radius=diameter / 2.0, height=height, sections=48)
    # cylinder default axis is +Z; align to direction
    z = np.array([0, 0, 1.0])
    if np.allclose(z, direction):
        pass
    elif np.allclose(z, -direction):
        cyl.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0]))
    else:
        cyl.apply_transform(trimesh.geometry.align_vectors(z, direction, return_transform=True))
    cyl.apply_translation(center)
    return link_mesh.difference(cyl, engine='manifold'), surface


def load_joint_plan(value, joints_pkl):
    """Resolve which joint plan to apply.

    Priority:
      1. --joint-plan <file.json>  (explicit plan file)
      2. --joint-plan <name>       (built-in preset from JOINT_PLANS)
      3. <model_stem>_joint_plan.json next to the joints pkl (convention)
      4. built-in preset whose name occurs in the model stem
    To support a new model, drop a <model_stem>_joint_plan.json next to its
    <model_stem>_joints.pkl — no code change needed.

    Plan file format (JSON list):
      [{"joint": "neck",  "type": "magnet", "parent": "BODY", "child": "HEAD"},
       {"joint": "l_hip", "type": "peg", "parent": "BODY", "child": "L_LEG",
        "depth_child": 6.0, "axis": [0, 0, -1]}]
    Keys: joint (annotation name), type (peg|magnet), parent, child,
    optional depth_parent / depth_child (mm, default 11.5), axis (direction
    override, default: shared joint -> farthest other joint of the child).
    """
    if value and os.path.isfile(value):
        with open(value) as f:
            return json.load(f), value
    if value and value in JOINT_PLANS:
        return JOINT_PLANS[value], f'built-in preset {value!r}'
    if value:
        raise SystemExit(f"unknown joint plan '{value}': neither a preset "
                         f"({sorted(JOINT_PLANS)}) nor an existing JSON file")
    stem = os.path.basename(joints_pkl).replace('_joints.pkl', '')
    conv = os.path.join(os.path.dirname(joints_pkl), stem + '_joint_plan.json')
    if os.path.isfile(conv):
        with open(conv) as f:
            return json.load(f), conv
    for name in JOINT_PLANS:
        if name in stem.lower():
            return JOINT_PLANS[name], f'built-in preset {name!r} (matched by model stem)'
    raise SystemExit(f"no joint plan for model '{stem}': pass --joint-plan <preset|plan.json> "
                     f"or create {conv}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--parts-mm', required=True)
    ap.add_argument('--joints-pkl', required=True)
    ap.add_argument('--source-stl', required=True, help='STL the annotations were made on')
    ap.add_argument('--expected-x', type=float, default=100.0, help='mm, as used in run.py')
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--magnet-diameter', type=float, default=9.0)
    ap.add_argument('--magnet-depth', type=float, default=4.0)
    ap.add_argument('--joint-plan', default=None,
                    help='preset name (see JOINT_PLANS) or path to a *_joint_plan.json; '
                         'default: auto-detect from the model')
    args = ap.parse_args()

    plan, plan_source = load_joint_plan(args.joint_plan, args.joints_pkl)
    print(f'joint plan: {plan_source}')

    os.makedirs(args.out_dir, exist_ok=True)
    src = trimesh.load(args.source_stl)
    unit_scale = args.expected_x / (src.bounds[1][0] - src.bounds[0][0])
    print(f'unit_scale (pkl -> mm): {unit_scale:.4f}')

    link_info = collect_link_info(load_link_tree(args.joints_pkl))
    for name in link_info:
        link_info[name]['joints'] = {jn: jp * unit_scale for jn, jp in link_info[name]['joints'].items()}

    links = {}
    for f in os.listdir(args.parts_mm):
        if f.lower().endswith('.stl'):
            mesh = trimesh.load(os.path.join(args.parts_mm, f))
            # decomposed parts at fine voxel sizes can contain degenerate
            # zero-volume slivers (2-face shells) that make manifold booleans
            # reject the mesh ("not all meshes are volumes") — keep the
            # largest component up front
            parts = mesh.split(only_watertight=False)
            if isinstance(parts, trimesh.Trimesh):
                parts = [parts]
            if len(parts) > 1:
                parts = sorted(parts, key=lambda p: p.volume, reverse=True)
                print(f'  {f}: dropped {len(parts) - 1} degenerate components '
                      f'(largest dropped: {parts[1].volume:.3f} mm3)')
                mesh = parts[0]
            links[f[:-4]] = mesh
    print('links loaded:', sorted(links.keys()))

    report = []
    hole_registry = {}  # link_name -> list of (joint_pos, direction, depth, label)

    for cfg in plan:
        jname, jtype = cfg['joint'], cfg['type']
        parent_name, child_name = cfg['parent'], cfg['child']
        jpos = link_info[child_name]['joints'][jname]

        # direction into the child link: shared joint -> child's farthest other joint
        if 'axis' in cfg:
            dir_child = np.asarray(cfg['axis'], dtype=float)
            dir_child /= np.linalg.norm(dir_child)
        else:
            others = [(jn, jp) for jn, jp in link_info[child_name]['joints'].items() if jn != jname]
            far_jn, far_jp = max(others, key=lambda t: np.linalg.norm(t[1] - jpos))
            dir_child = far_jp - jpos
            dir_child /= np.linalg.norm(dir_child)

        if jtype == 'magnet':
            parent, psurf = drill_magnet_hole(links[parent_name], jpos, args.magnet_diameter,
                                              args.magnet_depth, direction=-dir_child)
            child, csurf = drill_magnet_hole(links[child_name], jpos, args.magnet_diameter,
                                             args.magnet_depth, direction=dir_child)
            links[parent_name], links[child_name] = parent, child
            report.append({'joint': jname, 'type': jtype,
                           'parent_surface': psurf.round(2).tolist(),
                           'child_surface': csurf.round(2).tolist()})
            print(f'[magnet] {jname}: parent surface {psurf.round(2)}, child surface {csurf.round(2)}')
            continue

        # peg holes on both parent and child.
        # The hole must open at the link's ACTUAL cut face, which is not
        # necessarily at the annotated joint point: ray-cast from outside
        # the link along the hole axis and start the cutter at the surface.
        d_parent = cfg.get('depth_parent', PEG_FULL_DEPTH)
        d_child = cfg.get('depth_child', PEG_FULL_DEPTH)

        def surface_point(link_mesh, direction):
            origin = jpos - direction * 30.0
            ray = trimesh.ray.ray_triangle.RayMeshIntersector(link_mesh)
            locs, _, _ = ray.intersects_location([origin], [direction])
            if len(locs) == 0:
                raise RuntimeError(f'no surface along {direction} from {origin}')
            return locs[np.argmin((locs - origin) @ direction)]

        surf_p = surface_point(links[parent_name], -dir_child)
        surf_c = surface_point(links[child_name], dir_child)

        # full-depth holes: flip so the snap groove ("big end") is at the deep
        # end; truncated holes keep the groove inside the shallow hole as-is
        flip_p = d_parent >= PEG_FULL_DEPTH
        flip_c = d_child >= PEG_FULL_DEPTH
        cutter_p = peg_cutter(-dir_child, d_parent, flip=flip_p)
        cutter_p.apply_translation(surf_p)
        cutter_c = peg_cutter(dir_child, d_child, flip=flip_c)
        cutter_c.apply_translation(surf_c)
        # entry relief channels punch fully through sloped cut faces
        guide_p = peg_entry_relief(-dir_child, flip_p)
        guide_p.apply_translation(surf_p)
        guide_c = peg_entry_relief(dir_child, flip_c)
        guide_c.apply_translation(surf_c)

        parent = links[parent_name].difference([cutter_p, guide_p], engine='manifold')
        child = links[child_name].difference([cutter_c, guide_c], engine='manifold')
        links[parent_name], links[child_name] = parent, child

        hole_registry.setdefault(parent_name, []).append((surf_p, -dir_child, d_parent, f'{jname} (as parent)'))
        hole_registry.setdefault(child_name, []).append((surf_c, dir_child, d_child, f'{jname} (as child)'))
        entry = {'joint': jname, 'type': jtype, 'depth_parent': d_parent, 'depth_child': d_child,
                 'parent': {'link': parent_name, 'watertight': bool(parent.is_watertight),
                            'surface': surf_p.round(2).tolist()},
                 'child': {'link': child_name, 'watertight': bool(child.is_watertight),
                           'surface': surf_c.round(2).tolist()},
                 'peg': {'parent_entry': surf_p.round(2).tolist(), 'child_entry': surf_c.round(2).tolist(),
                         'dir_into_child': dir_child.round(4).tolist()}}
        report.append(entry)
        print(f'[peg] {jname}: {parent_name} watertight={parent.is_watertight} (depth {d_parent}), '
              f'{child_name} watertight={child.is_watertight} (depth {d_child})')

    # wall check between holes drilled from opposite ends of the same link
    print('\n-- hole wall check --')
    wall_report = {}
    for link_name, holes in hole_registry.items():
        if len(holes) < 2:
            continue
        for i in range(len(holes)):
            for j in range(i + 1, len(holes)):
                (j1, d1, dep1, lab1), (j2, d2, dep2, lab2) = holes[i], holes[j]
                delta = j2 - j1
                span = abs(delta @ d1)
                lateral = np.linalg.norm(delta - (delta @ d1) * d1)
                if lateral > 4.0:  # holes pass beside each other, no interference
                    wall_report[f'{link_name}: {lab1} <-> {lab2}'] = 'side-by-side (OK)'
                    print(f'  {link_name}: {lab1} <-> {lab2}: lateral offset {lateral:.1f} mm, side-by-side OK')
                    continue
                wall = span - dep1 - dep2
                key = f'{link_name}: {lab1} <-> {lab2}'
                wall_report[key] = round(wall, 2)
                flag = 'OK' if wall >= 2.0 else '!! THIN/INTERFERING !!'
                print(f'  {key}: span {span:.1f} - {dep1} - {dep2} = wall {wall:.1f} mm  {flag}')

    # remove disconnected fragments ("flying slivers") produced by the
    # booleans: keep only the largest connected component per link
    print('\n-- fragment cleanup --')
    fragment_report = {}
    for name, mesh in links.items():
        parts = mesh.split(only_watertight=False)
        if isinstance(parts, trimesh.Trimesh):
            parts = [parts]
        if len(parts) <= 1:
            continue
        parts = sorted(parts, key=lambda p: p.volume, reverse=True)
        removed = parts[1:]
        fragment_report[name] = {
            'removed_components': len(removed),
            'removed_volume_mm3': round(sum(p.volume for p in removed), 2),
            'largest_removed_mm3': round(removed[0].volume, 2),
        }
        print(f'  {name}: kept largest of {len(parts)} components, '
              f'removed {len(removed)} fragments '
              f'(total {fragment_report[name]["removed_volume_mm3"]} mm3, '
              f'largest {fragment_report[name]["largest_removed_mm3"]} mm3)')
        links[name] = parts[0]

    for name, mesh in links.items():
        mesh.export(os.path.join(args.out_dir, name + '.stl'))
    with open(os.path.join(args.out_dir, 'attach_report.json'), 'w') as f:
        json.dump({'joints': report, 'hole_walls_mm': wall_report,
                   'fragments_removed': fragment_report}, f, indent=2)
    print('saved to', args.out_dir)


if __name__ == '__main__':
    main()
