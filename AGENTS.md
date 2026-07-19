# Agent Guide for something2robot

This doc is for AI agents (and humans) who want to run the `something2robot` pipeline without reading the whole codebase.

## What this project does

Takes a 3D mesh (`.stl`) plus joint annotations (`_joints.pkl`) and produces:

- A URDF with movable joints
- Printable STL parts in **millimeters**
- Joint interface geometry (motor shells or magnet pockets)
- Motor placement visualization (motor mode only)
- A `report.json` summary

## Quick start (the only command you need)

```bash
cd /Users/zhaoshuai/Documents/workspace/something2robot
uv run python run.py --model <model_name> --expected-x <mm> --voxel-size <mm>
```

Example:

```bash
# Motor mode (actuated joints with motor shells)
uv run python run.py --model maneki_neko --expected-x 50 --voxel-size 0.5 --seed 42 --out-dir result/maneki_neko_motor

# Magnet mode (passive joints with flat interface + magnet pockets)
uv run python run.py --model maneki_neko --expected-x 50 --voxel-size 0.5 --connector-mode magnet --magnet-diameter 6 --magnet-thickness 2 --out-dir result/maneki_neko_magnet
```

## Environment

- Python 3.11 (locked in `.python-version`)
- Dependencies managed by `uv` (`pyproject.toml` + `uv.lock`)
- Do **not** use `pip install` globally; use `uv add` if you need a new package.

## `run.py` CLI reference

| Flag | Default | Description |
|------|---------|-------------|
| `--model` | required | Model name prefix. Must match an `.stl` in `auto_design/model/given_models/` |
| `--expected-x` | 100.0 | Target print size along X axis in **mm** |
| `--voxel-size` | 1.0 | Voxel size in **mm**. Smaller = finer but much slower |
| `--voxel-density` | 1.2e-4 | Material density in kg/cm³. Lower = lighter model, lower motor torque |
| `--seed` | 42 | Random seed for reproducibility |
| `--genetic-generation` | 5 | GA generations for motor placement (motor mode only) |
| `--max-trial-round` | 8 | How many times auto_design retries on failure |
| `--repair` | false | Keep only largest connected component for broken links |
| `--skip-motors` | false | Skip motor visualization export |
| `--connector-mode` | `motor` | Joint interface type: `motor`, `magnet`, or `none` |
| `--magnet-diameter` | 6.0 | Magnet diameter in mm (magnet mode only) |
| `--magnet-thickness` | 2.0 | Magnet pocket depth in mm (magnet mode only) |
| `--magnet-clearance` | 0.2 | Added diametral pocket clearance in mm (magnet mode only) |
| `--out-dir` | `result/<model>` | Output directory |

### Connector modes

| Mode | Joint type | Status | Geometry | Use case |
|------|-----------|--------|----------|----------|
| `motor` | `revolute` | ✅ stable | Motor shells + cylindrical cavity | Actuated robots with servos |
| `magnet` | `revolute` | ✅ stable | Flat interface ⟂ axis + magnet pockets | Passive turntable joints |
| `none` | `fixed` | ✅ stable | Plain split, jagged boundary | Debugging / raw decomposition |
| `pin` | `revolute` | 🔧 planned | Flat interface + through‑hole for metal pin | Passive hinge joints (cheapest) |
| `bearing` | `revolute` | 🔧 planned | Flat interface + bearing pocket + through‑hole | Passive hinge joints (smoothest) |

## Passive Joint Taxonomy

The project supports (or will support) several physical joint types for
non-actuated (passive) connections.  Choosing the right one depends on the
model's articulation style and desired mechanical properties.

### Two articulation topologies

```
  Turntable (转盘式)                   Hinge (铰链式)
  轴 ⟂ 身体表面                        轴 ∥ 身体表面

     旋转轴 (Y)                             旋转轴 (X)
        ↑                                     ←→
    ┌───┼───┐                            ═══════════
    │ arm    │                            BODY  │ leg
    │  ☉     │  ← 水平面上转动                │  ☉
    ├────────┤                            ═══════════
    │ BODY   │                              腿前后摆动
    └────────┘
     切面 ⟂ 轴 (水平)                      切面 ∥ 轴 (竖直)

  Example: maneki_neko arm               Example: bulldog leg
```

The **current flattening code assumes turntable topology** (plane ⟂ rotation
axis).  This is correct for maneki_neko but wrong for bulldog legs.  See
"Joint interface flattening" below for the planned fix.

### Joint type comparison

| Type | Cost/joint | Rotation | Printability | Reliability | Best for |
|------|-----------|----------|-------------|-------------|----------|
| **Magnet** | ¥2–5 | smooth | ★ easy | ★★ (can detach) | Quick prototypes |
| **Pin** (filament / wire) | ¥0 | smooth | ★ easy | ★★★★★ | Production, kids toys |
| **Bearing + pin** | ¥1–2 | ★★★★★ | ★★ medium | ★★★★★ | High-end, frequent use |
| **Ball joint** | N/A | 3‑DOF | ★★★ hard | ★★ | ❌ not recommended for printing |
| **Hinge (knuckle)** | ¥0 + pin | smooth | ★★★ hard | ★★★★★ | Heavy load joints |

### Why ball joints are not planned

3‑DOF ball joints (spherical rotation in any direction) present three
fundamental problems for 3D‑printed, magnet‑held assemblies:

1.  **Printability** — spherical sockets have massive overhangs requiring
    support material that ruins the mating surface.
2.  **Capture** — a ball‑and‑socket cannot be printed as one piece; it needs
    a multi‑part snap‑fit or bolt‑together enclosure.
3.  **Magnet placement** — magnet orientation on a spherical surface varies
    with rotation angle, so there is no single stable magnet position.

For practical robotics, 2‑DOF can be achieved by chaining two 1‑DOF pin
joints in series (e.g., a hip with two orthogonal pins).

## Model requirements

A usable model needs **both** files in `auto_design/model/given_models/`:

```
<name>.stl
<name>_joints.pkl
```

If the joints pkl is missing, `run.py` will fail and list available models. To create joints for a new model, use the UI:

```bash
uv run python script/auto_design.py \
  --stl_mesh_path auto_design/model/given_models/<name>.stl \
  --joint_pkl_path auto_design/model/given_models/<name>_joints.pkl \
  --expected_x 100 \
  --voxel_size 1.0
```

In the UI:

1. **Link Edit**: add child links (e.g. legs) under `BODY`.
2. **Joint Edit**: each link needs at least 2 joints. Shared joint with parent must have the **same name and coordinates** on both links.
3. **Axis Edit**: each non-root link needs an axis `[(x,y,z), (dx,dy,dz)]` defining its rotation axis. Root link can use `[(x,y,z), (0,0,0)]`.
4. Click **Save**, then close the UI.

## Output structure

```
<out-dir>/
├── <model>_<timestamp>/     # raw auto_design output
│   └── result_roundN/
│       ├── urdf/
│       │   ├── robot.urdf
│       │   └── *.stl
│       └── robot_result.pkl
├── parts/                    # copied URDF + meter-scale STLs
│   ├── robot.urdf
│   └── *.stl
├── parts_mm/                 # mm-scale STLs for printing
│   └── *.stl
├── motors/                   # motor placement STLs (motor mode only)
│   ├── motor_*.stl
│   └── motors_combined.mm.stl
└── report.json               # config, timings, exit_code, link checks
```

All results go under `result/` in the project root. Each run creates a subdirectory.

## Interpreting results

- `success: true` + `exit_code: 0` → fully validated, ready to print.
- `exit_code: 2` → destruction check failed, but parts/motors may still be generated.
- `exit_code: 3` → mesh not watertight; use `--repair` to keep largest component.
- `exit_code: 555` → exception during auto_design; check `notes` in `report.json`.

## Minimal MVP workflow

1. Pick or add a model with joints in `auto_design/model/given_models/`.
2. Run `run.py` with appropriate size/resolution and `--connector-mode`.
3. Check `report.json` for `exit_code` and `link_checks`.
4. Import `parts_mm/*.stl` into OrcaSlicer and print.
5. For motor mode, use `motors/*.stl` as a guide for where to insert physical motors.

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `No motor in the library meets the torque requirement` | Lower `--voxel-density` (e.g. `1e-5`) or use a smaller/lighter model |
| `The mesh is destroyed` (exit_code 2) | Increase `--expected-x` or use finer `--voxel-size`; the auto-scaler will retry if `--max-trial-round > 1` |
| Process hangs in destruction check | Use larger `--voxel-size` or `--max-trial-round 1` to avoid repeated checks |
| UI does not open for joint annotation | Make sure you are on macOS with a display; do not pass `--disable_joint_setting_ui` |
| Head/body parts misassigned to wrong link | Usually caused by the nearest-segment rule when a link passes close to another body part (e.g. maneki_neko's raised arm hugs the face). Disconnected misassigned voxels are now reassigned to the closest other link instead of being deleted (see `mesh_decomp.py`). If the main cluster itself is wrong, adjust joint positions in the UI. |
| Magnet pockets look conical / interface not flat | Use `--connector-mode magnet` for flat mating surfaces |

## Removed / unavailable (do not use)

This repo is a stripped-down fork of the upstream `anything2robot` project. The following were removed and **must not be used or relied on**:

| Feature / file | Status |
|----------------|--------|
| Ansys FEA (`--do_fea_analysis`) | **Unavailable.** The imports are commented out in `script/auto_design.py`; enabling it raises `NameError`. Keep it `False` (this is hardcoded in `run.py`). Failure code 4 can no longer occur. |
| Metamaterial filling (`metamaterial_filling/`) | **Removed from the repo.** All `sys.path` entries pointing to it are dead. |
| `script/fill_in_metamaterial_for_urdf.py` | **Broken** (imports the removed `metamaterial_filling` package). |
| `script/quadruped_auto_design_test.py`, `script/quadruped_success_sample_fea.py` | **Broken** (import removed modules `quadruped_pose_to_pkl` / `metamaterial_filling`). Legacy from the quadruped era. |
| `script/result_analysis.py`, `script/mesh_rotation.py` | Legacy standalone tools, **not part of the `run.py` pipeline** and unmaintained. Use at your own risk. |
| `script/boolean_magnet_pockets.py` | Standalone tool, **not wired into `run.py`**. Requires a Blender installation (`engine="blender"`), which is not declared in `pyproject.toml`. Magnet pockets in the normal pipeline are made by `--connector-mode magnet` (voxel-based), not by this script. |

## Files an agent should know about

| File | Purpose |
|------|---------|
| `run.py` | Single entry point for the whole pipeline |
| `script/auto_design.py` | Core auto-design logic, also exposes UI mode |
| `script/motor_param_lib.py` | Motor library (torque, size) |
| `script/pin_joint_demo.py` | Standalone post‑processing: add pin/bearing holes to STL parts |
| `auto_design/modules/motor_opt.py` | GA motor optimization + joint interface geometry (`_flatten_joint_interface`, SVM, magnet pockets, motor shells) |
| `auto_design/modules/mesh_decomp.py` | Voxel decomposition into links (nearest‑segment, DBSCAN, reassign) |
| `auto_design/modules/interference_removal.py` | Rotational interference removal + URDF generation |
| `auto_design/modules/urdf_generator.py` | URDF writing helpers |
| `auto_design/modules/mesh_loader.py` | STL loading, scaling, joint pkl I/O, LinkTree UI |
| `script/export_stl_to_mm.py` | Scales STL files to millimeters |
| `script/check_and_repair_links.py` | Checks link STL connectivity |
| `script/export_motor_visualization.py` | Exports motor placement STLs |
| `auto_design/model/given_models/` | Where input STL + joints pkl live |

## Future Integration Roadmap

Summary of planned connector‑mode and annotation upgrades, in priority order.

| Priority | Feature | Effort | Dependencies |
|----------|---------|--------|-------------|
| **P0** | `--connector-mode pin` — voxel‑based through‑hole | Small (adapt existing cylinder‑carve code) | None |
| **P0** | Per‑joint `cut_plane_normal` in annotations | Medium (UI + pkl + `_flatten_joint_interface`) | None |
| **P1** | `--connector-mode bearing` — voxel blind pocket + through‑hole | Small (pin mode + blind cylinder) | P0 pin mode |
| **P1** | Automatic rotation limiter / tenon generation | Medium (arc geometry in voxels) | Per‑joint rotation range metadata |
| **P2** | Auto‑detect cut plane direction (centroid / segment fallback) | Small | P0 annotation (as fallback) |
| **P2** | Hinge (knuckle) joint geometry | Large (interleaved teeth + pin hole) | P0 pin mode |
| **—** | Ball joint (3‑DOF) | Not planned | Printability issues, see Passive Joint Taxonomy |

### Boolean vs Voxel trade‑off reference

| Operation | trimesh boolean | Voxel pipeline |
|----------|----------------|----------------|
| Through‑hole on simple part | ✅ stable | ✅ stable |
| Through‑hole on complex part (many holes) | ❌ breaks after 2–3 ops | ✅ stable |
| Blind pocket (bearing / magnet) | ❌ non‑watertight output | ✅ stable |
| Complex interlocking (hinge knuckle) | ❌ almost impossible | ⚠️ feasible but complex |
| Preserves original mesh detail | ✅ | ❌ (limited by voxel resolution) |

## Joint interface flattening (magnet / none / pin mode)

When the pipeline runs in a non‑motor mode, `_flatten_joint_interface()` in
`auto_design/modules/motor_opt.py` cuts a flat interface between parent and
child links at each joint.

### Current behavior

The cutting plane is placed at the joint center with its **normal aligned to
the rotation axis** — i.e., the plane is perpendicular to the axis.  This
works correctly for **turntable** joints (maneki_neko arm, lamp head).

### Known limitation

For **hinge** joints (bulldog legs, corgi legs) the rotation axis points
horizontally through the body.  A plane perpendicular to this axis would
slice the body horizontally — separating top from bottom rather than leg
from body.  These models need a cut plane whose normal is aligned with the
link's long axis (joint‑to‑joint segment) or the parent→child centroid
direction.

On the initial bulldog run with `--connector-mode magnet`, the legs
separate correctly but the interface surface around the hip joint shows a
**concave hemispherical depression**.  This is *not* a bug — the SVM plane
is only applied inside a sphere around the joint center; outside that
sphere the original nearest‑segment decomposition preserves the body's
natural curved contour.  For hinge joints this concavity is actually
beneficial (it provides lateral constraint), but the overall cut direction
still needs to be corrected for a clean separation.

### Four candidate strategies for auto‑detecting cut direction

| # | Strategy | Works for turntable | Works for hinge | Requires |
|---|----------|--------------------|-----------------|----------|
| 1 | 3D linear SVM (no projection) | ✅ auto | ✅ auto | More voxels in sphere |
| 2 | Use link's joint‑to‑joint segment as normal | ✅ (fallback) | ✅ | Link must have ≥2 joints |
| 3 | Parent→child centroid in sphere as normal | ✅ auto | ✅ auto | Sensitive to sphere radius |
| 4 | User‑annotated `cut_plane_normal` in pkl | ✅ explicit | ✅ explicit | UI / pkl schema change |

### Planned direction

**Phase 1**: Add optional `cut_plane_normal` field to the joint annotation
format (both UI and pkl).  When present, use it directly; when absent, fall
back to strategy 3 (centroid).

**Phase 2**: Expose the cut plane direction in the joint‑setting UI so users
can preview and adjust without editing the pkl by hand.

### Rotation limiter / tenon structure

Motor mode naturally produces a rotation limiter: the SVM‑split cylindrical
motor shells collide when the joint rotates past ~180°, acting as a
physical stop.

Magnet / pin mode currently has **no built‑in limiter** — parts can rotate
infinitely, which risks magnet detachment or loose feel.

**Current workaround**: add arc‑shaped tenon structures in the slicer via
negative volumes.

```
  child 侧                           parent 侧
  ┌─────────────┐                  ┌─────────────┐
  │    ╭───╮    │  ← 弧形凸台       │  ╭───────╮  │  ← 弧形槽
  │    │   │    │     (正空间)      │  │       │  │     (负空间)
  │    ╰───╯    │                  │  ╰───────╯  │
  └─────────────┘                  └─────────────┘
     凸台在槽内滑动，弧两端 = 限位
```

**Planned automation**: once per‑joint rotation‑range metadata exists in
the annotation format, the pipeline can generate tenon geometry
automatically in voxel space.

### Pin joint demo (`script/pin_joint_demo.py`)

A standalone post‑processing script that adds cylindrical through‑holes to
already‑split STL files produced by a `--connector-mode magnet` run.

```bash
uv run python script/pin_joint_demo.py                                \
  --parts-mm      result/<model>_flat/parts_mm                        \
  --robot-result  result/<model>_flat/<ts>/result_round1/robot_result.pkl \
  --mode pin --pin-diameter 3                                         \
  --out-dir result/<model>_pin
```

**Results** (tested on maneki_neko and bulldog):
- Small parts (leg segments, arm): watertight ✅
- BODY (large part with 4–8 holes): fails after 2–3 holes ❌

This pattern confirms the trade‑off: **trimesh booleans are reliable for
simple parts with few modifications, but break on complex meshes with
many operations**.  The long‑term fix is to integrate pin holes into the
voxel pipeline (where the same hole geometry is carved on the voxel grid
before marching cubes).

## Notes

- Keep `package://anything2robot` URDF paths working by ensuring the `anything2robot -> .` symlink exists at repo root. `run.py` creates it automatically.
- Do not run destructive git commands (commit/push/rebase) unless explicitly asked.
- For reproducibility, always set `--seed`.
- All output now goes under `result/` in the project root by default.
- Unit convention: the `run.py` CLI takes millimetres (`--expected-x`, `--voxel-size`, magnet params) and converts them internally; the auto-design core (`script/auto_design.py`, `auto_design/modules/`) works in centimetres, and the generated URDF is in metres. Don't mix the two conventions when calling core functions directly.
