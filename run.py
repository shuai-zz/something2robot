import os
import sys
import argparse
import json
import time
import shutil
import re

project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)
sys.path.insert(0, os.path.join(project_root, 'auto_design'))
sys.path.insert(0, os.path.join(project_root, 'auto_design', 'modules'))
sys.path.insert(0, os.path.join(project_root, 'script'))
sys.path.insert(0, os.path.join(project_root, 'metamaterial_filling', 'script'))

from auto_design import auto_design_function, str2bool
from export_stl_to_mm import export_urdf_folder_to_mm
from check_and_repair_links import check_urdf_folder_links
from export_motor_visualization import export_motors_from_pkl

GIVEN_MODELS_DIR = os.path.join(project_root, 'auto_design', 'model', 'given_models')


def list_available_models():
    """Return a list of (stem, stl_path, pkl_path) for models with both .stl and _joints.pkl."""
    models = []
    if not os.path.isdir(GIVEN_MODELS_DIR):
        return models
    for filename in sorted(os.listdir(GIVEN_MODELS_DIR)):
        if not filename.lower().endswith('.stl'):
            continue
        stem = filename[:-4]
        pkl_name = stem + '_joints.pkl'
        stl_path = os.path.join(GIVEN_MODELS_DIR, filename)
        pkl_path = os.path.join(GIVEN_MODELS_DIR, pkl_name)
        if os.path.isfile(pkl_path):
            models.append((stem, stl_path, pkl_path))
    return models


def resolve_model(model_name):
    """Resolve a model name. Prefer exact stem match, then prefix (case-insensitive). Return (stem, stl_path, pkl_path)."""
    models = list_available_models()
    query = model_name.lower()

    # 1. Exact match (case-insensitive)
    exact = [(stem, stl, pkl) for stem, stl, pkl in models if stem.lower() == query]
    if len(exact) == 1:
        return exact[0]

    # 2. Prefix match
    prefix_matches = [(stem, stl, pkl) for stem, stl, pkl in models if stem.lower().startswith(query)]
    if len(prefix_matches) == 1:
        return prefix_matches[0]
    if len(prefix_matches) > 1:
        print(f"Error: model prefix '{model_name}' is ambiguous. Matches: {[m[0] for m in prefix_matches]}")
        _print_available_models(models)
        sys.exit(1)

    # 3. Substring fallback
    substring_matches = [(stem, stl, pkl) for stem, stl, pkl in models if query in stem.lower()]
    if len(substring_matches) == 1:
        return substring_matches[0]
    if len(substring_matches) > 1:
        print(f"Error: model name '{model_name}' is ambiguous. Matches: {[m[0] for m in substring_matches]}")
        _print_available_models(models)
        sys.exit(1)

    print(f"Error: no model matching '{model_name}' found.")
    _print_available_models(models)
    sys.exit(1)


def _print_available_models(models):
    print("\nAvailable models (need both .stl and _joints.pkl):")
    if not models:
        print("  (none)")
        return
    for stem, _, _ in models:
        print(f"  - {stem}")


def ensure_anything2robot_symlink():
    """Ensure the root symlink anything2robot -> . exists for package:// URDF paths."""
    link_path = os.path.join(project_root, 'anything2robot')
    if os.path.islink(link_path):
        target = os.readlink(link_path)
        if os.path.abspath(target) == project_root:
            return
        print(f"Warning: replacing existing symlink {link_path} -> {target}")
        os.remove(link_path)
    elif os.path.exists(link_path):
        print(f"Warning: {link_path} exists and is not a symlink; package://anything2robot paths may fail")
        return
    os.symlink('.', link_path)
    print(f"Created symlink: {link_path} -> .")


def copy_parts_with_relative_urdf(src_urdf_folder, dst_parts_folder):
    """Copy URDF and STL files to dst_parts_folder, rewrite mesh paths to relative filenames."""
    os.makedirs(dst_parts_folder, exist_ok=True)
    urdf_files = [f for f in os.listdir(src_urdf_folder) if f.endswith('.urdf')]
    if not urdf_files:
        raise FileNotFoundError(f"No URDF file found in {src_urdf_folder}")
    src_urdf_path = os.path.join(src_urdf_folder, urdf_files[0])
    dst_urdf_path = os.path.join(dst_parts_folder, 'robot.urdf')

    with open(src_urdf_path, 'r', encoding='utf-8') as f:
        urdf_text = f.read()

    pattern = r'package://anything2robot/[^"\'\']*/([^"\'\'/]+\.stl)'
    rewritten = re.sub(pattern, r'\1', urdf_text)

    with open(dst_urdf_path, 'w', encoding='utf-8') as f:
        f.write(rewritten)

    copied_stls = []
    for filename in os.listdir(src_urdf_folder):
        if filename.lower().endswith('.stl'):
            src = os.path.join(src_urdf_folder, filename)
            dst = os.path.join(dst_parts_folder, filename)
            shutil.copy2(src, dst)
            copied_stls.append(filename)

    return dst_urdf_path, copied_stls


def find_best_round_folder(result_folder, model_stem):
    """Find the best result_round folder under result_folder/<model_stem>_*."""
    if not os.path.isdir(result_folder):
        return None

    run_folders = []
    for entry in os.listdir(result_folder):
        full_path = os.path.join(result_folder, entry)
        if not os.path.isdir(full_path):
            continue
        if entry.startswith(model_stem + '_'):
            run_folders.append((os.path.getmtime(full_path), full_path))

    if not run_folders:
        return None

    # Most recent run folder first
    run_folders.sort(key=lambda x: x[0], reverse=True)
    latest_run = run_folders[0][1]

    round_folders = []
    for entry in os.listdir(latest_run):
        full_path = os.path.join(latest_run, entry)
        if not os.path.isdir(full_path):
            continue
        if entry.startswith('result_round'):
            try:
                round_num = int(entry[len('result_round'):])
            except ValueError:
                round_num = 0
            round_folders.append((round_num, full_path))

    if not round_folders:
        return None

    round_folders.sort(key=lambda x: x[0])

    # Prefer the first round containing a filename with 'exit_code_0'
    for _, round_path in round_folders:
        for _, _, files in os.walk(round_path):
            for fname in files:
                if 'exit_code_0' in fname:
                    return round_path

    # Otherwise use the last round
    return round_folders[-1][1]


class AutoDesignArgs:
    """Simple namespace for auto_design_function arguments. Defined at module level so it can be pickled."""
    pass


def build_args(stl_path, joints_path, out_dir, expected_x, voxel_size, seed,
               genetic_generation=5, max_trial_round=8, voxel_density=1.2e-4):
    args = AutoDesignArgs()
    args.stl_mesh_path = os.path.abspath(stl_path)
    args.joint_pkl_path = os.path.abspath(joints_path)
    args.result_folder = os.path.abspath(out_dir)
    args.expected_x = expected_x
    args.voxel_size = voxel_size
    args.voxel_density = voxel_density
    args.joint_limitation = 0.5
    args.joint_limitation_from_champ = True
    args.max_trial_round = max_trial_round
    args.genetic_generation = genetic_generation
    args.do_fea_analysis = False
    args.regenerate_if_fea_failed = False
    args.visualize = False
    args.disable_joint_setting_ui = True
    args.joint_setting_standard_scale = False
    args.model_name = 'None'
    args.seed = seed
    return args


def main():
    parser = argparse.ArgumentParser(
        description='Single agent-friendly entry point for something2robot.'
    )
    parser.add_argument('--model', type=str, required=True,
                        help='Model name prefix (case-insensitive), e.g. lamp or Cactus')
    parser.add_argument('--expected-x', type=float, default=100.0,
                        help='Expected x-axis length in mm (default: 100.0)')
    parser.add_argument('--voxel-size', type=float, default=1.0,
                        help='Voxel size in mm (default: 1.0)')
    parser.add_argument('--seed', type=int, default=42,
                        help='Random seed (default: 42)')
    parser.add_argument('--genetic-generation', type=int, default=5,
                        help='Genetic algorithm generations (default: 5)')
    parser.add_argument('--out-dir', type=str, default=None,
                        help='Output directory (default: result_agent_<model>)')
    parser.add_argument('--repair', action='store_true',
                        help='Repair disconnected STL links by keeping largest component')
    parser.add_argument('--skip-motors', action='store_true',
                        help='Skip motor visualization export')
    parser.add_argument('--max-trial-round', type=int, default=8,
                        help='Maximum auto-design trial rounds (default: 8)')
    parser.add_argument('--voxel-density', type=float, default=1.2e-4,
                        help='Voxel density in kg/cm^3 (default: 1.2e-4). Lower value reduces mass and motor torque requirements.')
    args_cli = parser.parse_args()

    start_time = time.time()
    report = {
        'model_input': args_cli.model,
        'expected_x_mm': args_cli.expected_x,
        'voxel_size_mm': args_cli.voxel_size,
        'voxel_density': args_cli.voxel_density,
        'seed': args_cli.seed,
        'genetic_generation': args_cli.genetic_generation,
        'max_trial_round': args_cli.max_trial_round,
        'repair': args_cli.repair,
        'skip_motors': args_cli.skip_motors,
        'project_root': project_root,
        'timings': {},
        'paths': {},
        'exit_code': None,
        'success': False,
        'notes': [],
    }

    # Resolve model
    try:
        model_stem, stl_path, joints_path = resolve_model(args_cli.model)
        print(f"Resolved model: {model_stem}")
        print(f"  STL:   {stl_path}")
        print(f"  Joints: {joints_path}")
        report['model_stem'] = model_stem
        report['paths']['stl'] = stl_path
        report['paths']['joints_pkl'] = joints_path
    except SystemExit:
        raise
    except Exception as e:
        report['notes'].append(f"Model resolution failed: {e}")
        _write_report(report, args_cli.out_dir or f"result_agent_{args_cli.model}")
        raise

    # Determine output directory
    out_dir = args_cli.out_dir or f"result_agent_{model_stem}"
    out_dir = os.path.abspath(out_dir)
    os.makedirs(out_dir, exist_ok=True)
    report['paths']['out_dir'] = out_dir

    # Ensure symlink
    try:
        ensure_anything2robot_symlink()
    except Exception as e:
        report['notes'].append(f"Symlink setup failed: {e}")

    # Build args and run auto_design
    args = build_args(
        stl_path=stl_path,
        joints_path=joints_path,
        out_dir=out_dir,
        expected_x=args_cli.expected_x,
        voxel_size=args_cli.voxel_size,
        seed=args_cli.seed,
        genetic_generation=args_cli.genetic_generation,
        max_trial_round=args_cli.max_trial_round,
        voxel_density=args_cli.voxel_density,
    )

    design_start = time.time()
    exit_code = -1
    try:
        import random
        import numpy as np
        random.seed(args_cli.seed)
        np.random.seed(args_cli.seed)
        exit_code = auto_design_function(args)
    except Exception as e:
        report['notes'].append(f"auto_design_function raised: {e}")
        exit_code = -2
    design_elapsed = time.time() - design_start
    report['exit_code'] = exit_code
    report['success'] = (exit_code == 0)
    report['timings']['auto_design_seconds'] = design_elapsed

    # Locate the best round folder
    round_folder = find_best_round_folder(out_dir, model_stem)
    report['paths']['round_folder'] = round_folder
    if not round_folder:
        report['notes'].append("No result round folder found under output directory.")
        _write_report(report, out_dir)
        print(json.dumps(report, indent=2))
        return

    print(f"Using round folder: {round_folder}")

    # Copy URDF and STLs to parts/
    parts_folder = os.path.join(out_dir, 'parts')
    src_urdf_folder = os.path.join(round_folder, 'urdf')
    if not os.path.isdir(src_urdf_folder):
        report['notes'].append(f"Round folder has no urdf/ subfolder; auto_design likely failed before URDF generation.")
        _write_report(report, out_dir)
        print(json.dumps(report, indent=2))
        return

    try:
        urdf_path, stl_files = copy_parts_with_relative_urdf(src_urdf_folder, parts_folder)
        report['paths']['parts_folder'] = parts_folder
        report['paths']['robot_urdf'] = urdf_path
        report['parts_stl_files'] = stl_files
    except Exception as e:
        report['notes'].append(f"Copy parts failed: {e}")

    # Scale to mm
    parts_mm_folder = os.path.join(out_dir, 'parts_mm')
    try:
        exported_mm = export_urdf_folder_to_mm(parts_folder, parts_mm_folder, 1000.0)
        report['paths']['parts_mm_folder'] = parts_mm_folder
        report['parts_mm_stl_files'] = exported_mm
    except Exception as e:
        report['notes'].append(f"export_stl_to_mm failed: {e}")

    # Check/repair links
    try:
        link_checks = check_urdf_folder_links(parts_mm_folder, repair=args_cli.repair)
        report['link_checks'] = link_checks
    except Exception as e:
        report['notes'].append(f"check_urdf_folder_links failed: {e}")
        report['link_checks'] = []

    # Export motors
    motors_folder = os.path.join(out_dir, 'motors')
    if not args_cli.skip_motors:
        pkl_path = os.path.join(round_folder, 'robot_result.pkl')
        if not os.path.isfile(pkl_path):
            report['notes'].append(f"robot_result.pkl not found; skipping motor export.")
        else:
            try:
                motor_files = export_motors_from_pkl(pkl_path, motors_folder, unit='mm')
                report['paths']['motors_folder'] = motors_folder
                report['motor_files'] = motor_files
            except Exception as e:
                report['notes'].append(f"export_motors_from_pkl failed: {e}")
    else:
        report['notes'].append("Motor export skipped by --skip-motors.")

    report['timings']['total_seconds'] = time.time() - start_time
    _write_report(report, out_dir)
    print(json.dumps(report, indent=2))


def _write_report(report, out_dir):
    report_path = os.path.join(out_dir, 'report.json')
    os.makedirs(out_dir, exist_ok=True)
    with open(report_path, 'w', encoding='utf-8') as f:
        json.dump(report, f, indent=2)
    print(f"Report saved to: {report_path}")


if __name__ == '__main__':
    main()
