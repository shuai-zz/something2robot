"""End-to-end joint pipeline: voxel decomposition (run.py) + peg/magnet attach.

One command, reproducible output:

    uv run python run_joints.py \
        --model Mario_Character_Image_1020080024_scaled \
        --expected-x 100 --voxel-size 1.0 --seed 42 \
        --out-dir result/mario_full

What it does:
  1. run.py --connector-mode none   -> <out-dir>/parts_mm (decomposed links)
  2. script/attach_joint_library.py -> <out-dir>/parts_jointed
       (keyed peg holes at hips/knees/ankles, magnet holes at the neck,
        entry relief channels, fragment cleanup, wall report)
  3. writes <out-dir>/run_joints_report.json with params, seed, git rev,
     per-step timings and the attach report for reproducibility.

Use --reuse-decomposition to skip step 1 when <out-dir>/parts_mm already
exists (iterate on the joint attach without re-running the decomposition).

Environment note (this machine only): if the venv's .pth files carry the
macOS hidden flag, pinocchio and the Qt cocoa plugin are skipped by their
loaders. The env workarounds below are applied automatically when present
and are harmless on normal machines:
  - cmeel.prefix site-packages appended to PYTHONPATH
  - QT_PLUGIN_PATH=/tmp/s2r_qtplugins/Qt5/plugins if that directory exists
    (recreate with: cp -R .venv/lib/python3.11/site-packages/PyQt5/Qt5/plugins \
       /tmp/s2r_qtplugins/Qt5/plugins && ln -s .venv/.../Qt5/lib /tmp/s2r_qtplugins/Qt5/lib)
"""

import argparse
import glob
import json
import os
import subprocess
import sys
import time

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
GIVEN_MODELS = os.path.join(PROJECT_ROOT, 'auto_design', 'model', 'given_models')


def resolve_model(query):
    """Same matching rule as run.py: exact stem, then unique prefix, then substring."""
    query = query.lower()
    models = []
    for f in sorted(os.listdir(GIVEN_MODELS)):
        if not f.lower().endswith('.stl'):
            continue
        stem = f[:-4]
        pkl = os.path.join(GIVEN_MODELS, stem + '_joints.pkl')
        if os.path.isfile(pkl):
            models.append((stem, os.path.join(GIVEN_MODELS, f), pkl))
    for candidates in (
        [m for m in models if m[0].lower() == query],
        [m for m in models if m[0].lower().startswith(query)],
        [m for m in models if query in m[0].lower()],
    ):
        if len(candidates) == 1:
            return candidates[0]
        if len(candidates) > 1:
            raise SystemExit(f"ambiguous model '{query}': {[m[0] for m in candidates]}")
    raise SystemExit(f"no model matching '{query}' in {GIVEN_MODELS}")


def workaround_env():
    env = os.environ.copy()
    cmeel = glob.glob(os.path.join(
        PROJECT_ROOT, '.venv', 'lib', 'python*', 'site-packages',
        'cmeel.prefix', 'lib', 'python*', 'site-packages'))
    if cmeel:
        env['PYTHONPATH'] = cmeel[0] + os.pathsep + env.get('PYTHONPATH', '')
    qt = '/tmp/s2r_qtplugins/Qt5/plugins'
    if os.path.isdir(qt):
        env['QT_PLUGIN_PATH'] = qt
    return env


def git_rev():
    try:
        return subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD'],
                                       cwd=PROJECT_ROOT, text=True).strip()
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--model', required=True)
    ap.add_argument('--expected-x', type=float, default=100.0, help='mm')
    ap.add_argument('--voxel-size', type=float, default=1.0, help='mm')
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--magnet-diameter', type=float, default=9.0)
    ap.add_argument('--magnet-depth', type=float, default=4.0)
    ap.add_argument('--joint-plan', default=None,
                    help='joint plan for the attach step: preset name (mario/cactus) or path '
                         'to a *_joint_plan.json; default: auto-detect (convention file next '
                         'to the joints pkl, else preset matched by model stem)')
    ap.add_argument('--reuse-decomposition', action='store_true',
                    help='skip run.py if <out-dir>/parts_mm already exists')
    args = ap.parse_args()

    stem, stl_path, pkl_path = resolve_model(args.model)
    out_dir = os.path.abspath(args.out_dir)
    parts_mm = os.path.join(out_dir, 'parts_mm')
    parts_jointed = os.path.join(out_dir, 'parts_jointed')
    os.makedirs(out_dir, exist_ok=True)
    env = workaround_env()

    report = {
        'model': stem, 'stl': stl_path, 'joints_pkl': pkl_path,
        'expected_x_mm': args.expected_x, 'voxel_size_mm': args.voxel_size,
        'seed': args.seed, 'git_rev': git_rev(), 'steps': {},
    }
    print(f'== model: {stem}\n== out:  {out_dir}')

    # ---- step 1: decomposition ------------------------------------------------
    t0 = time.time()
    if args.reuse_decomposition and os.path.isdir(parts_mm):
        print('\n== step 1/2: decomposition SKIPPED (reusing existing parts_mm)')
        report['steps']['decomposition'] = {'skipped': True}
    else:
        print('\n== step 1/2: decomposition (run.py, connector-mode none)')
        cmd = [sys.executable, os.path.join(PROJECT_ROOT, 'run.py'),
               '--model', stem,
               '--expected-x', str(args.expected_x),
               '--voxel-size', str(args.voxel_size),
               '--seed', str(args.seed),
               '--connector-mode', 'none',
               '--max-trial-round', '1',
               '--out-dir', out_dir]
        r = subprocess.run(cmd, env=env, cwd=PROJECT_ROOT)
        dt = time.time() - t0
        report['steps']['decomposition'] = {'seconds': round(dt, 1), 'returncode': r.returncode}
        if r.returncode != 0:
            print(f'!! decomposition failed (rc={r.returncode}), see output above')
            sys.exit(r.returncode)
        print(f'== decomposition done in {dt:.0f}s')

    # ---- step 2: joint attach --------------------------------------------------
    t0 = time.time()
    print('\n== step 2/2: joint attach (peg holes + magnet holes)')
    cmd = [sys.executable, os.path.join(PROJECT_ROOT, 'script', 'attach_joint_library.py'),
           '--parts-mm', parts_mm,
           '--joints-pkl', pkl_path,
           '--source-stl', stl_path,
           '--expected-x', str(args.expected_x),
           '--out-dir', parts_jointed,
           '--magnet-diameter', str(args.magnet_diameter),
           '--magnet-depth', str(args.magnet_depth)]
    if args.joint_plan:
        cmd += ['--joint-plan', args.joint_plan]
    r = subprocess.run(cmd, env=env, cwd=PROJECT_ROOT)
    dt = time.time() - t0
    report['steps']['joint_attach'] = {'seconds': round(dt, 1), 'returncode': r.returncode}
    if r.returncode != 0:
        print(f'!! joint attach failed (rc={r.returncode})')
        sys.exit(r.returncode)
    print(f'== joint attach done in {dt:.0f}s')

    # fold the attach report into the run report
    attach_report_path = os.path.join(parts_jointed, 'attach_report.json')
    if os.path.isfile(attach_report_path):
        with open(attach_report_path) as f:
            report['attach'] = json.load(f)

    report_path = os.path.join(out_dir, 'run_joints_report.json')
    with open(report_path, 'w') as f:
        json.dump(report, f, indent=2)
    print(f'\n== done. jointed parts: {parts_jointed}\n== report: {report_path}')


if __name__ == '__main__':
    main()
