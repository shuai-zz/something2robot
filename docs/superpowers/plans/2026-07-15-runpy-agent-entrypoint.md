# run.py Agent 入口实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 新增单一入口 `run.py`，让外部 agent 用一条命令完成从 given model 到可打印 mm 单位 STL + 电机可视化的完整流程。

**Architecture：** `run.py` 作为编排器，调用现有 `script/auto_design.py` 的 `auto_design_function`，再调用重构后的导出/检查脚本函数，把结果整理成固定目录结构并生成 `report.json`。不改动 auto_design 核心逻辑，只暴露函数接口。

**Tech Stack：** Python 3.11, trimesh, numpy, uv, argparse, pathlib

---

## 文件结构

| 文件 | 职责 |
|------|------|
| `script/export_stl_to_mm.py` | 新增 `export_urdf_folder_to_mm()` 函数；保留 CLI |
| `script/check_and_repair_links.py` | 新增 `check_urdf_folder_links()` 函数，返回结构化结果；保留 CLI |
| `script/export_motor_visualization.py` | 新增 `export_motors_from_pkl()` 函数；保留 CLI |
| `run.py` | 新增 agent 入口，编排整个流程 |

---

### Task 1: 重构 `script/export_stl_to_mm.py`，暴露 `export_urdf_folder_to_mm()`

**Files:**
- Modify: `script/export_stl_to_mm.py`
- Test: 手动运行 `uv run python script/export_stl_to_mm.py --urdf_folder ...`

- [ ] **Step 1: 在文件顶部导入后添加核心函数**

```python
def export_urdf_folder_to_mm(urdf_folder, output_folder, scale=1000.0):
    """把 urdf_folder 里的 STL 缩放到毫米并输出到 output_folder，返回导出的文件列表。"""
    os.makedirs(output_folder, exist_ok=True)
    exported = []
    for filename in sorted(os.listdir(urdf_folder)):
        if not filename.endswith('.stl'):
            continue
        src_path = os.path.join(urdf_folder, filename)
        mesh = trimesh.load(src_path)
        mesh.apply_scale(scale)
        dst_path = os.path.join(output_folder, filename)
        mesh.export(dst_path)
        exported.append(filename)
    return exported
```

- [ ] **Step 2: 修改 `main()` 复用新函数**

把 `main()` 里从 `os.makedirs` 到 `print(f"\n共导出...")` 的循环逻辑替换为：

```python
    exported = export_urdf_folder_to_mm(urdf_folder, output_folder, args.scale)
    for filename in exported:
        print(f"  {filename:20s} 已导出")
    print(f"\n共导出 {len(exported)} 个 STL 到 {output_folder}")
```

- [ ] **Step 3: 验证 CLI 仍可用**

Run:
```bash
uv run python script/export_stl_to_mm.py \
  --urdf_folder result_lamp_seed42_gen20_x30_v04/lamp_scaled_20260715-153127/result_round1/urdf \
  --output_folder /tmp/test_mm
```

Expected: 正常导出，无异常。

---

### Task 2: 重构 `script/check_and_repair_links.py`，暴露 `check_urdf_folder_links()`

**Files:**
- Modify: `script/check_and_repair_links.py`
- Test: 手动运行 `uv run python script/check_and_repair_links.py --urdf_folder ...`

- [ ] **Step 1: 修改导入并新增核心函数**

在文件顶部改为：

```python
import os
import argparse
import trimesh


def check_urdf_folder_links(folder, repair=False, output_suffix='_repaired'):
    """检查 URDF 文件夹里每个 link STL 的连通性，返回结构化列表。"""
    stl_files = [
        f for f in os.listdir(folder)
        if f.endswith('.stl') and not f.endswith(output_suffix + '.stl')
    ]
    results = []
    for f in sorted(stl_files):
        path = os.path.join(folder, f)
        mesh = trimesh.load(path)
        components = mesh.split(only_watertight=False)
        n = len(components)
        results.append({
            'file': f,
            'components': n,
            'watertight': bool(mesh.is_watertight),
            'repaired': False,
        })
        if n > 1 and repair:
            largest = max(components, key=lambda c: len(c.faces))
            out_path = os.path.join(folder, f.replace('.stl', output_suffix + '.stl'))
            largest.export(out_path)
            results[-1]['repaired'] = True
    return results
```

- [ ] **Step 2: 修改 `check_links()` 复用新函数并保留打印**

```python
def check_links(folder, repair=False, output_suffix='_repaired'):
    """检查 URDF 文件夹里每个 link STL 的连通性，可选只保留最大连通块。"""
    results = check_urdf_folder_links(folder, repair=repair, output_suffix=output_suffix)
    print(f"检查 {len(results)} 个 STL 文件...\n")
    broken = []
    for r in results:
        status = "✅ 连通" if r['components'] == 1 else f"⚠️  断开成 {r['components']} 个部分"
        print(f"{r['file']:20s} {status:20s} watertight={r['watertight']}")
        if r['components'] > 1:
            broken.append(r['file'])
            if r['repaired']:
                print(f"  -> 已修复并保存: {r['file'].replace('.stl', output_suffix + '.stl')}")
    print(f"\n总结: {len(broken)} 个文件断开" + ("，已修复" if repair and broken else ""))
    return results
```

- [ ] **Step 3: 验证 CLI 仍可用**

Run:
```bash
uv run python script/check_and_repair_links.py \
  --urdf_folder result_lamp_seed42_gen20_x30_v04/lamp_scaled_20260715-153127/result_round1/urdf
```

Expected: 正常打印连通性结果。

---

### Task 3: 重构 `script/export_motor_visualization.py`，暴露 `export_motors_from_pkl()`

**Files:**
- Modify: `script/export_motor_visualization.py`
- Test: 手动运行 `uv run python script/export_motor_visualization.py --pkl_path ...`

- [ ] **Step 1: 在 `main()` 之前新增核心函数**

```python
def export_motors_from_pkl(pkl_path, output_folder=None, unit='mm'):
    """从 robot_result.pkl 导出电机可视化 STL，返回导出的文件列表。"""
    if output_folder is None:
        output_folder = os.path.join(os.path.dirname(pkl_path), 'motors')
    os.makedirs(output_folder, exist_ok=True)

    scale = 1000.0 if unit == 'mm' else 1.0
    cm_to_out = 10.0 if unit == 'mm' else 0.01

    rr = pickle.load(open(pkl_path, 'rb'))
    combined = []
    motor_idx_counter = {}
    exported = []

    for link_name, link_result in rr.link_dict.items():
        for i, tenon_type in enumerate(link_result.tenon_type):
            tenon_pos = np.array(link_result.tenon_pos[i][:3])
            tenon_dir = np.array(link_result.tenon_pos[i][3:6])
            tenon_dir = tenon_dir / np.linalg.norm(tenon_dir)
            tenon_idx = link_result.tenon_idx[i]

            motor_height_cm, motor_radius_cm, _ = motor_lib[tenon_idx]
            radius = motor_radius_cm * cm_to_out
            height = motor_height_cm * cm_to_out

            center = tenon_pos * scale + height * 0.5 * tenon_dir
            cylinder = trimesh.creation.cylinder(radius=radius, height=height, sections=32)
            rot = rotation_matrix_from_vectors(np.array([0, 0, 1]), tenon_dir)
            transform = np.eye(4)
            transform[:3, :3] = rot
            transform[:3, 3] = center
            cylinder.apply_transform(transform)

            colors = [[255, 0, 0], [0, 255, 0], [0, 0, 255], [255, 255, 0], [0, 255, 255]]
            color = colors[tenon_idx % len(colors)]
            cylinder.visual.vertex_colors = color

            count = motor_idx_counter.get((link_name, tenon_idx), 0)
            filename = f"motor_{link_name}_{tenon_type}_{count}.stl"
            motor_idx_counter[(link_name, tenon_idx)] = count + 1
            cylinder.export(os.path.join(output_folder, filename))
            exported.append(filename)
            combined.append(cylinder)

    if combined:
        scene = trimesh.util.concatenate(combined)
        combined_name = f"motors_combined.{unit.lower()}.stl"
        scene.export(os.path.join(output_folder, combined_name))
        exported.append(combined_name)

    return exported
```

- [ ] **Step 2: 修改 `main()` 复用新函数**

```python
def main():
    parser = argparse.ArgumentParser(description='从 robot_result.pkl 导出电机可视化 STL，用于查看电机应该插在哪里。')
    parser.add_argument('--pkl_path', type=str, required=True, help='robot_result.pkl 路径')
    parser.add_argument('--output_folder', type=str, default=None, help='输出文件夹')
    parser.add_argument('--unit', type=str, default='mm', choices=['m', 'mm'], help='输出单位')
    args = parser.parse_args()

    exported = export_motors_from_pkl(args.pkl_path, args.output_folder, args.unit)
    for filename in exported:
        print(f"导出: {filename}")
    print(f"\n所有电机 STL 已保存到: {args.output_folder or os.path.join(os.path.dirname(args.pkl_path), 'motors')}")
```

- [ ] **Step 3: 验证 CLI 仍可用**

Run:
```bash
uv run python script/export_motor_visualization.py \
  --pkl_path result_lamp_seed42_gen20_x30_v04/lamp_scaled_20260715-153127/result_round1/robot_result.pkl \
  --output_folder /tmp/test_motors
```

Expected: 正常导出电机 STL。

---

### Task 4: 实现 `run.py`

**Files:**
- Create: `run.py`
- Test: `uv run python run.py --model lamp --expected-x 30 --voxel-size 0.4 --seed 42 --out-dir result_agent_lamp_test`

- [ ] **Step 1: 创建 `run.py` 骨架**

```python
import os
import sys
import argparse
import json
import time
import shutil
import re
from pathlib import Path

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
```

- [ ] **Step 2: 实现模型解析函数**

```python
GIVEN_MODELS_DIR = os.path.join(project_root, 'auto_design', 'model', 'given_models')


def list_available_models():
    """返回 given_models 目录下所有可用的模型名（不含 _joints 等后缀）。"""
    models = set()
    if not os.path.isdir(GIVEN_MODELS_DIR):
        return []
    for f in os.listdir(GIVEN_MODELS_DIR):
        if f.lower().endswith('.stl'):
            models.add(f[:-4])
    return sorted(models)


def resolve_model(model_name):
    """根据简称找到 .stl 和 _joints.pkl。"""
    if not os.path.isdir(GIVEN_MODELS_DIR):
        raise FileNotFoundError(f"找不到模型目录: {GIVEN_MODELS_DIR}")

    candidates = []
    for f in os.listdir(GIVEN_MODELS_DIR):
        if f.lower().endswith('.stl') and model_name.lower() in f.lower():
            candidates.append(f)

    if not candidates:
        available = list_available_models()
        raise FileNotFoundError(
            f"找不到包含 '{model_name}' 的 STL 模型。可用模型:\n" + "\n".join(f"  - {m}" for m in available)
        )

    # 优先完全匹配前缀
    candidates.sort(key=lambda x: (not x.lower().startswith(model_name.lower()), x))
    stl_file = candidates[0]
    stl_path = os.path.join(GIVEN_MODELS_DIR, stl_file)
    model_stem = stl_file[:-4]
    joints_file = model_stem + '_joints.pkl'
    joints_path = os.path.join(GIVEN_MODELS_DIR, joints_file)

    if not os.path.exists(joints_path):
        available = list_available_models()
        raise FileNotFoundError(
            f"找到 STL '{stl_file}'，但缺少 joints 文件 '{joints_file}'。"
            f"\n可用模型:\n" + "\n".join(f"  - {m}" for m in available)
        )

    return stl_path, joints_path, model_stem
```

- [ ] **Step 3: 实现软链检查函数**

```python
def ensure_anything2robot_symlink():
    """确保根目录存在 anything2robot -> . 软链，供 package://anything2robot 解析。"""
    link_path = os.path.join(project_root, 'anything2robot')
    if os.path.islink(link_path):
        return
    if os.path.exists(link_path):
        raise RuntimeError(f"'{link_path}' 已存在但不是软链，请手动处理。")
    os.symlink('.', link_path)
    print(f"创建软链: {link_path} -> .")
```

- [ ] **Step 4: 实现 URDF 路径重写函数**

```python
def copy_parts_with_relative_urdf(src_urdf_folder, dst_parts_folder):
    """复制 URDF 和 STL 到 parts/，并把 URDF 中的 mesh 路径改成本地相对路径。"""
    os.makedirs(dst_parts_folder, exist_ok=True)
    urdf_files = [f for f in os.listdir(src_urdf_folder) if f.endswith('.urdf')]
    if not urdf_files:
        raise FileNotFoundError(f"在 {src_urdf_folder} 中找不到 .urdf 文件")

    src_urdf_path = os.path.join(src_urdf_folder, urdf_files[0])
    dst_urdf_path = os.path.join(dst_parts_folder, 'robot.urdf')

    with open(src_urdf_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # 把 package://anything2robot/.../link.stl 替换为 link.stl
    content = re.sub(
        r'package://anything2robot/[^"\']*/([^"\'/]+\.stl)',
        r'\1',
        content
    )

    with open(dst_urdf_path, 'w', encoding='utf-8') as f:
        f.write(content)

    # 复制所有 STL
    copied = []
    for f in os.listdir(src_urdf_folder):
        if f.endswith('.stl'):
            shutil.copy2(os.path.join(src_urdf_folder, f), os.path.join(dst_parts_folder, f))
            copied.append(f)
    return copied
```

- [ ] **Step 5: 实现查找最佳 round 函数**

```python
def find_best_round_folder(result_folder, model_stem):
    """在 result_folder/<model_stem>_*/ 下找第一个 exit_code_0 的 round，否则找最后一个 round。"""
    prefix = model_stem + '_'
    model_folders = [
        os.path.join(result_folder, d)
        for d in os.listdir(result_folder)
        if d.startswith(prefix) and os.path.isdir(os.path.join(result_folder, d))
    ]
    if not model_folders:
        return None
    # 取最新的模型结果文件夹
    model_folder = max(model_folders, key=os.path.getmtime)

    round_folders = [
        os.path.join(model_folder, d)
        for d in os.listdir(model_folder)
        if d.startswith('result_round') and os.path.isdir(os.path.join(model_folder, d))
    ]
    round_folders.sort()

    for rf in round_folders:
        if any('exit_code_0' in f for f in os.listdir(rf)):
            return rf
    return round_folders[-1] if round_folders else None
```

- [ ] **Step 6: 实现 `build_args()` 和 `main()`**

```python
def build_args(stl_path, joints_path, out_dir, expected_x_mm, voxel_size_mm, seed, genetic_generation=5):
    """构造 auto_design_function 需要的 args 对象。"""
    class Args:
        pass
    args = Args()
    args.stl_mesh_path = os.path.abspath(stl_path)
    args.joint_pkl_path = os.path.abspath(joints_path)
    args.result_folder = os.path.abspath(out_dir)
    args.expected_x = expected_x_mm / 100.0  # mm -> m
    args.voxel_size = voxel_size_mm / 100.0  # mm -> m
    args.voxel_density = 1.2e-4
    args.joint_limitation = 0.5
    args.joint_limitation_from_champ = True
    args.max_trial_round = 8
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
    parser = argparse.ArgumentParser(description='Agent 友好的 something2robot 单一入口')
    parser.add_argument('--model', type=str, required=True, help='given_models 中的模型名（可前缀匹配）')
    parser.add_argument('--expected-x', type=float, default=100.0, help='目标打印尺寸（mm）')
    parser.add_argument('--voxel-size', type=float, default=1.0, help='体素大小（mm）')
    parser.add_argument('--seed', type=int, default=42, help='随机种子')
    parser.add_argument('--genetic-generation', type=int, default=5, help='遗传算法代数')
    parser.add_argument('--out-dir', type=str, default=None, help='输出目录')
    parser.add_argument('--repair', action='store_true', help='修复断开的连杆（保留最大连通块）')
    parser.add_argument('--skip-motors', action='store_true', help='跳过电机可视化导出')
    parser.add_argument('--max-trial-round', type=int, default=8, help='auto_design 最大尝试轮数')
    args_cli = parser.parse_args()

    out_dir = args_cli.out_dir or f"result_agent_{args_cli.model}"
    out_dir = os.path.abspath(out_dir)
    os.makedirs(out_dir, exist_ok=True)

    report = {
        'model': args_cli.model,
        'expected_x_mm': args_cli.expected_x,
        'voxel_size_mm': args_cli.voxel_size,
        'seed': args_cli.seed,
        'genetic_generation': args_cli.genetic_generation,
        'success': False,
        'exit_code': -1,
        'round': 1,
        'timings': {},
        'files': {},
        'link_checks': [],
        'notes': [],
    }

    ensure_anything2robot_symlink()
    stl_path, joints_path, model_stem = resolve_model(args_cli.model)
    report['stl_path'] = stl_path
    report['joints_path'] = joints_path

    print(f"模型: {model_stem}")
    print(f"STL: {stl_path}")
    print(f"Joints: {joints_path}")
    print(f"输出: {out_dir}")

    # Step 1: auto_design
    design_args = build_args(
        stl_path, joints_path, out_dir,
        args_cli.expected_x, args_cli.voxel_size,
        args_cli.seed, args_cli.genetic_generation
    )
    design_args.max_trial_round = args_cli.max_trial_round

    t0 = time.time()
    exit_code = auto_design_function(design_args)
    report['timings']['design_seconds'] = round(time.time() - t0, 2)
    report['exit_code'] = int(exit_code)
    report['success'] = (exit_code == 0)

    # Step 2: 定位结果
    round_folder = find_best_round_folder(out_dir, model_stem)
    if round_folder is None:
        report['notes'].append('找不到 result_round 文件夹')
        _save_report(out_dir, report)
        print("错误：找不到生成的 round 文件夹")
        return

    report['round'] = int(round_folder.split('result_round')[-1])
    src_urdf_folder = os.path.join(round_folder, 'urdf')
    pkl_path = os.path.join(round_folder, 'robot_result.pkl')

    # Step 3: 整理 parts/
    parts_folder = os.path.join(out_dir, 'parts')
    try:
        copied = copy_parts_with_relative_urdf(src_urdf_folder, parts_folder)
        report['files']['urdf'] = 'parts/robot.urdf'
        report['files']['parts'] = [f"parts/{f}" for f in copied]
    except Exception as e:
        report['notes'].append(f'整理 parts/ 失败: {e}')
        _save_report(out_dir, report)
        return

    # Step 4: 导出 parts_mm/
    parts_mm_folder = os.path.join(out_dir, 'parts_mm')
    t0 = time.time()
    try:
        mm_files = export_urdf_folder_to_mm(parts_folder, parts_mm_folder, scale=1000.0)
        report['files']['parts_mm'] = [f"parts_mm/{f}" for f in mm_files]
    except Exception as e:
        report['notes'].append(f'导出 mm 失败: {e}')
    report['timings']['export_mm_seconds'] = round(time.time() - t0, 2)

    # Step 5: 连杆连通性检查
    t0 = time.time()
    try:
        link_checks = check_urdf_folder_links(parts_mm_folder, repair=args_cli.repair)
        report['link_checks'] = link_checks
    except Exception as e:
        report['notes'].append(f'连杆检查失败: {e}')
    report['timings']['check_links_seconds'] = round(time.time() - t0, 2)

    # Step 6: 电机可视化
    if not args_cli.skip_motors:
        motors_folder = os.path.join(out_dir, 'motors')
        t0 = time.time()
        try:
            motor_files = export_motors_from_pkl(pkl_path, motors_folder, unit='mm')
            report['files']['motors'] = [f"motors/{f}" for f in motor_files]
        except Exception as e:
            report['notes'].append(f'电机导出失败: {e}')
        report['timings']['export_motors_seconds'] = round(time.time() - t0, 2)

    _save_report(out_dir, report)
    print("\n=== Report ===")
    print(json.dumps(report, indent=2, ensure_ascii=False))


def _save_report(out_dir, report):
    report_path = os.path.join(out_dir, 'report.json')
    with open(report_path, 'w', encoding='utf-8') as f:
        json.dump(report, f, indent=2, ensure_ascii=False)


if __name__ == '__main__':
    main()
```

- [ ] **Step 7: 语法检查**

Run:
```bash
uv run python -m py_compile run.py
```

Expected: 无输出，退出码 0。

---

### Task 5: 用 lamp 跑通完整流程

**Files:**
- Test: `run.py`

- [ ] **Step 1: 运行 lamp 测试**

Run:
```bash
uv run python run.py \
  --model lamp \
  --expected-x 30 \
  --voxel-size 0.4 \
  --seed 42 \
  --out-dir result_agent_lamp_test
```

Expected:
- 命令成功退出（exit code 0）。
- 生成 `result_agent_lamp_test/report.json`。
- 生成 `result_agent_lamp_test/parts/robot.urdf` 和若干 STL。
- 生成 `result_agent_lamp_test/parts_mm/`。
- 生成 `result_agent_lamp_test/motors/`。

- [ ] **Step 2: 检查 report.json**

Run:
```bash
cat result_agent_lamp_test/report.json
```

Expected:
- `success: true`
- `exit_code: 0`
- `files.parts_mm` 非空
- `files.motors` 非空
- `link_checks` 中有连通性记录

- [ ] **Step 3: 验证 URDF 路径已本地化**

Run:
```bash
grep -o 'package://[^"]*' result_agent_lamp_test/parts/robot.urdf | head
```

Expected: 无输出（所有 mesh 路径已改为相对路径）。

---

### Task 6: 用 cactus 验证模型匹配

**Files:**
- Test: `run.py`

- [ ] **Step 1: 运行 cactus 快速验证（不实际跑完，只验证解析）**

Run:
```bash
uv run python run.py --model cactus --expected-x 100 --voxel-size 1.0 --seed 42 --out-dir result_agent_cactus_test --max-trial-round 1 --genetic-generation 1
```

Expected:
- 模型解析成功，进入 auto_design。
- 由于 genetic_generation=1，过程较短，但仍需等待。
- 最终生成 report.json。

---

### Task 7: 文档更新

**Files:**
- Modify: `README.md` 或新增 `docs/agent_usage.md`

- [ ] **Step 1: 在 README.md 末尾添加 agent 用法**

```markdown
## Agent 快速入口

项目根目录提供 `run.py`，外部 agent 可一键运行：

```bash
uv run python run.py --model lamp --expected-x 30 --voxel-size 0.4 --seed 42 --out-dir result_agent_lamp
```

输出结构：

- `parts/`：原始 URDF 和米单位 STL
- `parts_mm/`：毫米单位 STL，可直接导入 OrcaSlicer
- `motors/`：电机位置可视化
- `report.json`：运行报告
```

---

## Self-Review

**Spec coverage:**
- 单一入口 `run.py`：Task 4
- 模型自动匹配：Task 4 `resolve_model`
- mm 导出：Task 1 + Task 4 Step 4
- 连杆连通性检查：Task 2 + Task 4 Step 5
- 电机可视化：Task 3 + Task 4 Step 6
- 固定输出目录 + report.json：Task 4
- 软链处理：Task 4 `ensure_anything2robot_symlink`

**Placeholder scan：** 所有步骤包含实际代码和命令，无 TBD/TODO。

**Type consistency：**
- `export_urdf_folder_to_mm` 返回 `List[str]`
- `check_urdf_folder_links` 返回 `List[dict]`
- `export_motors_from_pkl` 返回 `List[str]`
- 三个函数在 `run.py` 中用法一致。

**Gap：** 未包含单元测试，因为项目目前没有测试框架；验证依赖手动运行 lamp/cactus。
