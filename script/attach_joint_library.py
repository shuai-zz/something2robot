"""Attach printable, detachable joints to decomposed link STLs.

Joint plan: built-in presets (JOINT_PLANS below) or a JSON file per model —
see load_joint_plan() for resolution order and the file format. To support a
new model, add <model_stem>_joint_plan.json next to its _joints.pkl.
  magnet -> blind magnet holes on both links
  peg    -> keyed peg holes on both links
  ball   -> a recessed snap-fit ball stud on the child and socket in the parent

Peg hole cutter: auto_design/model/joint_models/peg_joint/7mm_keyed_peg_hole_cutter_clearance_0p30.stl
  - axis along +Z, entry at z=0 (small lip to z=-0.3), snap groove at z≈2.5..5,
    straight guide to z=11.5
  - keyed anti-rotation flats are perpendicular to the cutter Y axis
  - 0.3 mm clearance is already built into the cutter, use it directly for boolean

Placement rules:
  - hole axis = joint separation direction, drilled from the cut face INTO each link
    (parent gets the hole pointing away from the child and vice versa)
  - anti-rotation flats are kept parallel to the figure's chest/back plane
    (cutter Y axis mapped onto the model Y axis as closely as the hole axis allows)
  - when one link receives two holes from opposite ends, the holes can be
    shortened via depth overrides so a wall remains between them (see the
    wall report printed at the end)

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
                               '7mm_keyed_peg_hole_cutter_clearance_0p30.stl')
PEG_FULL_DEPTH = 11.5  # cutter spans z=0 (entry) .. z=11.5 (deep end)
HINGE_DIR = os.path.join(JOINT_MODELS_DIR, 'peg_joint')
BARE_HINGE_DIR = os.path.join(JOINT_MODELS_DIR, 'only_joint')

# Joint plans per model. depth_parent / depth_child override PEG_FULL_DEPTH.
JOINT_PLANS = {
    'mario': [
        {'joint': 'neck',    'type': 'magnet', 'parent': 'BODY',  'child': 'HEAD'},
        {'joint': 'l_hip',   'type': 'peg',    'parent': 'BODY',  'child': 'L_LEG'},
        {'joint': 'r_hip',   'type': 'peg',    'parent': 'BODY',  'child': 'R_LEG'},
        # knees removed 2026-07-29: lower legs merged into L_LEG / R_LEG, so the
        # leg now spans hip -> ankle; both ends take full-depth holes (wall ~17 mm)
        # ankles: drill straight down (vertical) instead of along the foot so the
        # peg can be inserted without interference
        {'joint': 'l_ankle', 'type': 'peg',    'parent': 'L_LEG', 'child': 'L_FOOT', 'axis': [0, 0, -1]},
        {'joint': 'r_ankle', 'type': 'peg',    'parent': 'R_LEG', 'child': 'R_FOOT', 'axis': [0, 0, -1]},
        # NOTE: hardware_bay for the 29.8 mm smart-hardware cube is implemented
        # but disabled — the 100 mm mario torso has no clean spot for it
        # (neck taper above, hip peg holes below). See AGENTS.md "Hardware bay".
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

    The cutter's snap groove (the "big end", z≈2.5..5 in library coords)
    sits near its z=0 end. For full-depth holes the groove must be at the
    DEEP end of the hole ("big end toward the model interior"): flip the
    cutter so the smooth guide enters first and the groove lands at
    z≈6.5..9. Short holes (depth overrides) keep the groove within the
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
    Keys: joint (annotation name), type (peg|magnet|ball|hinge|pin), parent, child,
    optional depth_parent / depth_child (mm, default 11.5), axis (direction
    override, default: shared joint -> farthest other joint of the child).
    Ball joints additionally accept ball_diameter, ball_clearance,
    neck_diameter, socket_mouth_ratio, socket_embed, and relief_slot (all
    dimensions in mm).
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


def ray_surface(mesh, point, direction):
    """First surface hit casting from OUTSIDE the mesh toward it along direction."""
    direction = np.asarray(direction, dtype=float)
    direction /= np.linalg.norm(direction)
    origin = point + direction * 80.0
    ray = trimesh.ray.ray_triangle.RayMeshIntersector(mesh)
    locs, _, _ = ray.intersects_location([origin], [-direction])
    if len(locs) == 0:
        raise RuntimeError(f'no surface along {direction} near {point}')
    return locs[np.argmin((locs - origin) @ -direction)]


def oriented_box(dims, rot, center):
    """Box with extents `dims`, axes rotated by rot, centered at center."""
    b = trimesh.creation.box(extents=list(dims))
    t = np.eye(4)
    t[:3, :3] = rot
    b.apply_transform(t)
    b.apply_translation(center)
    return b


def cylinder_along(direction, radius, height, center):
    """Cylinder of given radius/height whose axis follows direction, centered at center."""
    c = trimesh.creation.cylinder(radius=radius, height=height, sections=48)
    z = np.array([0, 0, 1.0])
    direction = np.asarray(direction, dtype=float)
    direction /= np.linalg.norm(direction)
    if np.allclose(z, direction):
        pass
    elif np.allclose(z, -direction):
        c.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0]))
    else:
        t = np.eye(4)
        t[:3, :3] = rotation_with_axis(direction)
        c.apply_transform(t)
    c.apply_translation(center)
    return c


def hidden_ball_joint(parent_mesh, child_mesh, parent_surface, child_surface,
                      direction_into_child, cfg):
    """Build a recessed, detachable ball joint.

    The ball and its neck are unioned to the child.  A matching spherical
    cavity and smaller retaining mouth are carved into the parent.  The ball
    center is recessed behind the parent cut face so the assembled joint shows
    only a small spherical cap instead of a PEG-like connector shaft.

    A narrow spring slot through the socket mouth is enabled by default.  It
    lets PETG/nylon socket jaws flex during insertion; set ``relief_slot`` to
    0 for a clean rigid socket or when the parent material is already flexible.
    """
    d_child = np.asarray(direction_into_child, dtype=float)
    d_child /= np.linalg.norm(d_child)
    d_parent = -d_child

    diameter = float(cfg.get('ball_diameter', 10.0))
    radius = diameter / 2.0
    clearance = float(cfg.get('ball_clearance', 0.35))
    neck_diameter = float(cfg.get('neck_diameter', diameter * 0.5))
    mouth_ratio = float(cfg.get('socket_mouth_ratio', 0.82))
    embed = float(cfg.get('socket_embed', radius * 0.75))
    relief_slot = float(cfg.get('relief_slot', 1.2))

    if diameter <= 0 or clearance < 0 or neck_diameter <= 0:
        raise ValueError('ball_diameter and neck_diameter must be positive; '
                         'ball_clearance must be non-negative')
    if not 0.55 <= mouth_ratio < 1.0:
        raise ValueError('socket_mouth_ratio must be in [0.55, 1.0)')
    if embed <= 0:
        raise ValueError('socket_embed must be positive')

    ball_center = np.asarray(parent_surface, dtype=float) + d_parent * embed
    ball = trimesh.creation.icosphere(subdivisions=3, radius=radius)
    ball.apply_translation(ball_center)

    # Start the neck slightly inside the child to guarantee a robust union
    # even when the decomposed cut surfaces do not coincide perfectly.
    neck_start = np.asarray(child_surface, dtype=float) + d_child * 0.8
    neck_vec = ball_center - neck_start
    neck_length = float(np.linalg.norm(neck_vec))
    if neck_length < 0.5:
        raise ValueError('ball joint neck is too short; check joint axis/surfaces')
    neck = cylinder_along(neck_vec, neck_diameter / 2.0,
                          neck_length + 1.0,
                          (neck_start + ball_center) / 2.0)
    child = child_mesh.union([neck, ball], engine='manifold')

    cavity = trimesh.creation.icosphere(
        subdivisions=3, radius=radius + clearance)
    cavity.apply_translation(ball_center)

    mouth_radius = max(neck_diameter / 2.0 + clearance,
                       radius * mouth_ratio)
    mouth_length = embed + radius + 4.0
    mouth_center = np.asarray(parent_surface, dtype=float) - d_parent * (
        (mouth_length - embed) / 2.0)
    mouth = cylinder_along(d_parent, mouth_radius, mouth_length, mouth_center)
    cutters = [cavity, mouth]

    if relief_slot > 0:
        rot = rotation_with_axis(d_parent)
        slot_depth = embed + radius + 3.0
        slot_center = np.asarray(parent_surface, dtype=float) + d_parent * (
            (slot_depth - 2.0) / 2.0)
        slot = oriented_box(
            [relief_slot, 2.0 * (radius + clearance + 2.0), slot_depth + 2.0],
            rot, slot_center)
        cutters.append(slot)

    parent = parent_mesh.difference(cutters, engine='manifold')
    info = {
        'ball_diameter': diameter,
        'ball_clearance': clearance,
        'neck_diameter': neck_diameter,
        'socket_mouth_ratio': mouth_ratio,
        'socket_embed': embed,
        'relief_slot': relief_slot,
        'ball_center': ball_center.round(3).tolist(),
    }
    return parent, child, info


def detachable_pin_joint(parent_mesh, child_mesh, parent_surface, child_surface,
                         direction_into_child, cfg):
    """Carve two blind coaxial holes and return a separate printable pin."""
    d_child = np.asarray(direction_into_child, dtype=float)
    d_child /= np.linalg.norm(d_child)
    diameter = float(cfg.get('pin_diameter', 3.0))
    clearance = float(cfg.get('pin_clearance', 0.2))
    depth_parent = float(cfg.get('depth_parent', 6.0))
    depth_child = float(cfg.get('depth_child', 6.0))
    tip_clearance = float(cfg.get('pin_tip_clearance', 0.35))
    if diameter <= 0 or clearance < 0 or min(depth_parent, depth_child) <= 0:
        raise ValueError('pin diameter/depths must be positive and clearance non-negative')
    if tip_clearance < 0 or 2 * tip_clearance >= depth_parent + depth_child:
        raise ValueError('pin_tip_clearance is incompatible with the pin depths')

    hole_radius = diameter / 2.0 + clearance

    def hole(surface, direction, depth):
        return cylinder_along(
            direction, hole_radius, depth + 1.0,
            np.asarray(surface, dtype=float) + direction * (depth / 2.0 - 0.25))

    parent = parent_mesh.difference(
        hole(parent_surface, -d_child, depth_parent), engine='manifold')
    child = child_mesh.difference(
        hole(child_surface, d_child, depth_child), engine='manifold')
    pin_length = depth_parent + depth_child - 2.0 * tip_clearance
    pin = trimesh.creation.cylinder(
        radius=diameter / 2.0, height=pin_length, sections=48)
    info = {
        'pin_diameter': diameter,
        'pin_clearance': clearance,
        'depth_parent': depth_parent,
        'depth_child': depth_child,
        'pin_tip_clearance': tip_clearance,
        'pin_length': pin_length,
    }
    return parent, child, pin, info


def hinge_connector(role, peg_depth):
    """Load a library hinge half and shorten its keyed insertion stem.

    Library convention: the stud stem extends from the bare hinge toward +Y;
    the socket stem extends toward -Y.  Cropping only the stem preserves the
    hinge knuckle and its pin/snap geometry.
    """
    if role not in ('stud', 'socket'):
        raise ValueError("hinge role must be 'stud' or 'socket'")
    if peg_depth <= 0 or peg_depth > PEG_FULL_DEPTH:
        raise ValueError(f'hinge peg depth must be in (0, {PEG_FULL_DEPTH}]')
    filename = f'hinge_joint_{role}.stl'
    mesh = trimesh.load(os.path.join(HINGE_DIR, filename))
    if peg_depth >= PEG_FULL_DEPTH:
        return mesh
    bare = trimesh.load(os.path.join(BARE_HINGE_DIR, filename))
    lo = mesh.bounds[0] - 2.0
    hi = mesh.bounds[1] + 2.0
    if role == 'stud':
        hi[1] = bare.bounds[1, 1] + peg_depth
    else:
        lo[1] = bare.bounds[0, 1] - peg_depth
    crop = trimesh.creation.box(extents=hi - lo)
    crop.apply_translation((hi + lo) / 2.0)
    return mesh.intersection(crop, engine='manifold')


def bay_metrics(mesh, pos, open_dir, R, size, lip, capsules):
    """Score a candidate bay position WITHOUT carving (pure ray casts +
    capsule math). Returns dict of min wall thicknesses in mm:
      front_min : shell in front of the cavity's front wall (grid over the
                  wall plane extended by the seat lip — catches edge slits)
      side_min  : shell outside the 4 lateral walls (3 samples each)
      hole_min  : clearance to existing peg/magnet hole capsules
    """
    ux, uy = R[:, 0], R[:, 1]
    out = {}

    def wall_thickness(wall_point, d):
        """Shell thickness from a cavity wall point outward along d.
        Casts from OUTSIDE the mesh so interior surfaces (holes) are ignored.
        Returns 0 when the wall point itself is outside the silhouette
        (cavity would poke out)."""
        wp = np.asarray(wall_point, dtype=float)
        if not mesh.contains([wp])[0]:
            return 0.0
        hit = ray_surface(mesh, wp, d)
        return float((hit - wp) @ d)

    # front wall (opposite the opening), grid over wall + lip extent
    front_plane = pos - open_dir * (size / 2.0)
    vals = []
    for i in np.linspace(-1, 1, 5):
        for j in np.linspace(-1, 1, 5):
            off = ux * (i * (size / 2 + lip)) + uy * (j * (size / 2 + lip))
            vals.append(wall_thickness(front_plane + off, -open_dir))
    out['front_min'] = round(min(vals), 2)

    # lateral walls, 3 samples each
    side_vals = []
    for d, u in [(R[:, 0], uy), (-R[:, 0], uy), (R[:, 1], ux), (-R[:, 1], ux)]:
        for k in np.linspace(-1, 1, 3):
            side_vals.append(wall_thickness(pos + d * (size / 2.0) + u * (k * size / 2.5), d))
    out['side_min'] = round(min(side_vals), 2)

    # clearance to existing holes: AABB-vs-AABB signed distance, with the hole
    # modelled as a flat-ended cylinder (radius r except along its own axis) —
    # a capsule model would overshoot by r at the blind end and flag phantom
    # collisions (e.g. with the neck magnet hole floor)
    def aabb_signed_gap(lo_a, hi_a, lo_b, hi_b):
        gaps = np.maximum(lo_a - hi_b, lo_b - hi_a)
        if np.all(gaps < 0):
            return float(np.max(gaps))  # overlapping: negative clearance
        return float(np.linalg.norm(np.maximum(gaps, 0)))

    half = np.abs(R[:, 0]) * size / 2 + np.abs(R[:, 1]) * size / 2 + np.abs(open_dir) * size / 2
    cav_lo, cav_hi = pos - half, pos + half
    hole_vals = []
    for (p1, p2, r, axis) in capsules:
        radial = 1.0 - np.abs(axis)
        hole_lo = np.minimum(p1, p2) - r * radial
        hole_hi = np.maximum(p1, p2) + r * radial
        hole_vals.append(aabb_signed_gap(cav_lo, cav_hi, hole_lo, hole_hi))
    out['hole_min'] = round(min(hole_vals), 2) if hole_vals else None
    return out


def find_bay_position(mesh, cfg, R, size, lip, capsules, open_dir):
    """Grid-search the bay position maximizing the worst wall thickness.
    Search ranges from cfg['search_center'] / cfg['search_y'] / cfg['search_z']
    (defaults tuned for a torso). Returns (best_pos, metrics)."""
    x0, y0, z0 = cfg.get('search_center', [0, 0, 0])
    xs = cfg.get('search_x', np.arange(x0 - 2, x0 + 3, 1.0))
    ys = cfg.get('search_y', np.arange(y0, y0 + 9, 1.0))
    zs = cfg.get('search_z', np.arange(z0 - 3, z0 + 7, 1.0))
    scored = []
    for z in zs:
        for y in ys:
            for x in xs:
                pos = np.array([x, y, z], dtype=float)
                m = bay_metrics(mesh, pos, open_dir, R, size, lip, capsules)
                score = min(m['front_min'], m['side_min'],
                            m['hole_min'] if m['hole_min'] is not None else 99)
                scored.append((score, pos, m))
    scored.sort(key=lambda t: -t[0])
    for score, pos, m in scored[:3]:
        print(f'  [hardware_bay] candidate {pos.tolist()} worst={score:.2f} {m}')
    best_pos, best_m = scored[0][1], scored[0][2]
    return best_pos, best_m


def hardware_bay(link_mesh, cfg, out_dir, link_name, capsules=None):
    """Carve a cubic hardware bay into link_mesh and build its friction cover.

    Steps (all manifold booleans):
      1. cubic cavity (size + 2*clearance)^3 at cfg['position'], or at an
         auto-searched position when cfg['position'] == 'auto' (maximizes the
         worst wall thickness via bay_metrics; capsules = existing holes to avoid)
      2. square opening channel from the cavity through the shell along
         cfg['open_direction'], plus a recessed seat for the cover
      3. friction cover (separate STL): plate sized to the seat + plug into
         the channel, optional center speaker hole
      4. optional speaker grille (grid of small holes) on the opposite side
    Returns (new_link_mesh, cover_mesh, info_dict).
    """
    open_dir = np.asarray(cfg['open_direction'], dtype=float)
    open_dir /= np.linalg.norm(open_dir)
    size = cfg.get('size', 29.8) + 2.0 * cfg.get('clearance', 0.4)
    lip = cfg.get('seat_lip', 2.0)
    seat_depth = cfg.get('seat_depth', 2.0)
    plug_depth = cfg.get('plug_depth', 2.0)
    R = rotation_with_axis(open_dir, flat_normal=np.array([0.0, 0.0, 1.0]))

    if isinstance(cfg.get('position'), str) and cfg['position'] == 'auto':
        pos, metrics = find_bay_position(link_mesh, cfg, R, size, lip,
                                         capsules or [], open_dir)
        print(f'  [hardware_bay] auto position {pos.tolist()} (metrics {metrics})')
    else:
        pos = np.asarray(cfg.get('position', [0, 0, 0]), dtype=float)

    surf = ray_surface(link_mesh, pos, open_dir)

    cavity = oriented_box([size, size, size], R, pos)
    chan_len = float((surf - pos) @ open_dir) - size / 2.0 + 8.0
    channel = oriented_box([size, size, chan_len], R, pos + open_dir * (size / 2.0 + chan_len / 2.0))
    seat = oriented_box([size + 2 * lip, size + 2 * lip, seat_depth + 4.0], R,
                        surf + open_dir * (4.0 - seat_depth) / 2.0)
    body = link_mesh.difference([cavity, channel, seat], engine='manifold')

    # speaker grille on the side opposite the opening
    g = cfg.get('speaker_grille')
    if g:
        gd, sp = g.get('diameter', 3.0), g.get('spacing', 5.0)
        rows, cols = g.get('rows', 3), g.get('cols', 3)
        ux, uy = R[:, 0], R[:, 1]
        base = pos - open_dir * (size / 2.0)
        cyls = []
        for i in range(rows):
            for j in range(cols):
                off = ux * (i - (rows - 1) / 2.0) * sp + uy * (j - (cols - 1) / 2.0) * sp
                cyls.append(cylinder_along(open_dir, gd / 2.0, 40.0, base + off - open_dir * 14.0))
        body = body.difference(cyls, engine='manifold')

    # friction cover, exported in assembled pose
    cover_plate = oriented_box([size + 2 * lip - 0.4, size + 2 * lip - 0.4, seat_depth], R,
                               surf - open_dir * (seat_depth / 2.0))
    cover_plug = oriented_box([size - 0.4, size - 0.4, plug_depth], R,
                              surf - open_dir * (seat_depth + plug_depth / 2.0))
    cover = cover_plate.union(cover_plug, engine='manifold')
    spk_d = cfg.get('speaker_back_diameter')
    if spk_d:
        cover = cover.difference(
            cylinder_along(open_dir, spk_d / 2.0, seat_depth + plug_depth + 4.0,
                           surf - open_dir * (seat_depth + plug_depth) / 2.0),
            engine='manifold')

    # post-carve verification with the same metric used for placement
    walls = bay_metrics(body, pos, open_dir, R, size, lip, capsules or [])

    info = {'type': 'hardware_bay', 'link': link_name, 'position': pos.tolist(),
            'open_direction': open_dir.tolist(), 'size': size,
            'surface': surf.round(2).tolist(), 'walls_mm': walls,
            'cover_watertight': bool(cover.is_watertight),
            'body_watertight': bool(body.is_watertight)}
    return body, cover, info


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
    hole_capsules = {}  # link_name -> list of (p1, p2, radius) for bay auto-placement

    def register_capsule(link_name, entry, direction, depth, radius):
        p1 = np.asarray(entry, dtype=float)
        d = np.asarray(direction, dtype=float)
        d /= np.linalg.norm(d)
        hole_capsules.setdefault(link_name, []).append((p1, p1 + d * depth, radius, d))

    for cfg in plan:
        jtype = cfg['type']

        if jtype == 'hardware_bay':
            link_name = cfg['link']
            body, cover, info = hardware_bay(links[link_name], cfg, args.out_dir, link_name,
                                             capsules=hole_capsules.get(link_name, []))
            links[link_name] = body
            cover_path = os.path.join(args.out_dir, f'{link_name}_hardware_cover.stl')
            cover.export(cover_path)
            report.append(info)
            print(f'[hardware_bay] {link_name}: body watertight={info["body_watertight"]}, '
                  f'cover watertight={info["cover_watertight"]}, walls {info["walls_mm"]}, '
                  f'cover -> {cover_path}')
            continue

        jname = cfg['joint']
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
            register_capsule(parent_name, psurf, -dir_child, args.magnet_depth, args.magnet_diameter / 2.0)
            register_capsule(child_name, csurf, dir_child, args.magnet_depth, args.magnet_diameter / 2.0)
            report.append({'joint': jname, 'type': jtype,
                           'parent_surface': psurf.round(2).tolist(),
                           'child_surface': csurf.round(2).tolist()})
            print(f'[magnet] {jname}: parent surface {psurf.round(2)}, child surface {csurf.round(2)}')
            continue

        if jtype == 'ball':
            surf_p = ray_surface(links[parent_name], jpos, dir_child)
            surf_c = ray_surface(links[child_name], jpos, -dir_child)
            parent, child, ball_info = hidden_ball_joint(
                links[parent_name], links[child_name],
                surf_p, surf_c, dir_child, cfg)
            links[parent_name], links[child_name] = parent, child
            report.append({
                'joint': jname,
                'type': jtype,
                'parent': {
                    'link': parent_name,
                    'watertight': bool(parent.is_watertight),
                    'surface': surf_p.round(2).tolist(),
                },
                'child': {
                    'link': child_name,
                    'watertight': bool(child.is_watertight),
                    'surface': surf_c.round(2).tolist(),
                },
                **ball_info,
            })
            print(f'[ball] {jname}: {parent_name} socket '
                  f'watertight={parent.is_watertight}, {child_name} stud '
                  f'watertight={child.is_watertight}, center '
                  f'{np.asarray(ball_info["ball_center"]).round(2)}')
            continue

        if jtype == 'pin':
            surf_p = ray_surface(links[parent_name], jpos, dir_child)
            surf_c = ray_surface(links[child_name], jpos, -dir_child)
            parent, child, pin, pin_info = detachable_pin_joint(
                links[parent_name], links[child_name],
                surf_p, surf_c, dir_child, cfg)
            links[parent_name], links[child_name] = parent, child
            pin_path = os.path.join(args.out_dir, f'{jname}_pin.stl')
            pin.export(pin_path)
            report.append({
                'joint': jname, 'type': jtype,
                'parent': {'link': parent_name, 'watertight': bool(parent.is_watertight)},
                'child': {'link': child_name, 'watertight': bool(child.is_watertight)},
                'pin_file': os.path.basename(pin_path),
                **pin_info,
            })
            print(f'[pin] {jname}: {parent_name}/{child_name} watertight='
                  f'{parent.is_watertight}/{child.is_watertight}, '
                  f'pin -> {pin_path}')
            continue

        if jtype not in ('peg', 'hinge'):
            raise ValueError(
                f"unsupported joint type {jtype!r} for joint {jname!r}; "
                "expected peg, magnet, ball, hinge, pin, or hardware_bay")

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
        # peg hole groove is 9.0 mm wide: capsule radius 4.5 for bay clearance
        register_capsule(parent_name, surf_p, -dir_child, d_parent, 4.5)
        register_capsule(child_name, surf_c, dir_child, d_child, 4.5)
        entry = {'joint': jname, 'type': jtype, 'depth_parent': d_parent, 'depth_child': d_child,
                 'parent': {'link': parent_name, 'watertight': bool(parent.is_watertight),
                            'surface': surf_p.round(2).tolist()},
                 'child': {'link': child_name, 'watertight': bool(child.is_watertight),
                           'surface': surf_c.round(2).tolist()},
                 'peg': {'parent_entry': surf_p.round(2).tolist(), 'child_entry': surf_c.round(2).tolist(),
                         'dir_into_child': dir_child.round(4).tolist()}}
        report.append(entry)
        print(f'[{jtype}] {jname}: {parent_name} watertight={parent.is_watertight} (depth {d_parent}), '
              f'{child_name} watertight={child.is_watertight} (depth {d_child})')

        if jtype == 'hinge':
            parent_half = hinge_connector('socket', d_parent)
            child_half = hinge_connector('stud', d_child)
            parent_file = f'{jname}_hinge_parent_socket.stl'
            child_file = f'{jname}_hinge_child_stud.stl'
            parent_half.export(os.path.join(args.out_dir, parent_file))
            child_half.export(os.path.join(args.out_dir, child_file))
            entry['hinge'] = {
                'parent_connector': parent_file,
                'child_connector': child_file,
                'parent_connector_watertight': bool(parent_half.is_watertight),
                'child_connector_watertight': bool(child_half.is_watertight),
            }

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
