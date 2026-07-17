# Agent Guide for something2robot

This doc is for AI agents (and humans) who want to run the `something2robot` pipeline without reading the whole codebase.

## What this project does

Takes a 3D mesh (`.stl`) plus joint annotations (`_joints.pkl`) and produces:

- A URDF with chosen motors
- Printable STL parts in **millimeters**
- Motor placement visualization
- A `report.json` summary

## Quick start (the only command you need)

```bash
cd /Users/zhaoshuai/Documents/workspace/something2robot
uv run python run.py --model <model_name> --expected-x <mm> --voxel-size <mm>
```

Example:

```bash
uv run python run.py --model lamp --expected-x 30 --voxel-size 0.4 --seed 42 --out-dir result_agent_lamp
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
| `--genetic-generation` | 5 | GA generations for motor placement |
| `--max-trial-round` | 8 | How many times auto_design retries on failure |
| `--repair` | false | Keep only largest connected component for broken links |
| `--skip-motors` | false | Skip motor visualization export |
| `--out-dir` | `result_agent_<model>` | Output directory |

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
├── motors/                   # motor placement STLs (mm)
│   ├── motor_*.stl
│   └── motors_combined.mm.stl
└── report.json               # config, timings, exit_code, link checks
```

## Interpreting results

- `success: true` + `exit_code: 0` → fully validated, ready to print.
- `exit_code: 2` → destruction check failed, but parts/motors may still be generated.
- `exit_code: 555` → exception during auto_design; check `notes` in `report.json`.

## Minimal MVP workflow

1. Pick or add a model with joints in `auto_design/model/given_models/`.
2. Run `run.py` with appropriate size/resolution.
3. Check `report.json` for `exit_code` and `link_checks`.
4. Import `parts_mm/*.stl` into OrcaSlicer and print.
5. Use `motors/*.stl` as a guide for where to insert physical motors.

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `No motor in the library meets the torque requirement` | Lower `--voxel-density` (e.g. `1e-5`) or use a smaller/lighter model |
| `The mesh is destroyed` (exit_code 2) | Increase `--expected-x` or use finer `--voxel-size`; the auto-scaler will retry if `--max-trial-round > 1` |
| Process hangs in destruction check | Use larger `--voxel-size` or `--max-trial-round 1` to avoid repeated checks |
| UI does not open for joint annotation | Make sure you are on macOS with a display; do not pass `--disable_joint_setting_ui` |

## Files an agent should know about

| File | Purpose |
|------|---------|
| `run.py` | Single entry point for the whole pipeline |
| `script/auto_design.py` | Core auto-design logic, also exposes UI mode |
| `script/export_stl_to_mm.py` | Scales STL files to millimeters |
| `script/check_and_repair_links.py` | Checks link STL connectivity |
| `script/export_motor_visualization.py` | Exports motor placement STLs |
| `script/motor_param_lib.py` | Motor library (torque, size) |
| `auto_design/model/given_models/` | Where input STL + joints pkl live |

## Notes

- Keep `package://anything2robot` URDF paths working by ensuring the `anything2robot -> .` symlink exists at repo root. `run.py` creates it automatically.
- Do not run destructive git commands (commit/push/rebase) unless explicitly asked.
- For reproducibility, always set `--seed`.
