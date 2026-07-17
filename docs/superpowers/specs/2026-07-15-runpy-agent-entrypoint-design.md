# run.py Agent 入口设计

## 日期
2026-07-15

## 背景
`something2robot` 当前的核心入口是 `script/auto_design.py`，参数较多，且输出目录结构不固定。外部 agent 调用时需要记住一长串路径、单位转换、电机导出等步骤，容易出错。本设计新增一个单一入口 `run.py`，把整个“从模型到可打印结果”的流程封装成一条命令。

## 目标
- 提供 agent 友好的单一 CLI 入口。
- 一条命令完成：模型选择 → auto_design → mm 导出 → 连杆连通性检查 → 电机可视化 → 生成报告。
- 输出目录结构固定，便于 agent 解析。
- 保持对现有代码的最小侵入，不拆分 Qt UI。

## 非目标
- 不替换 `script/auto_design.py` 原有 CLI，只在其上做包装。
- 不做结构校验、ANSYS FEA、模型填充等高级任务。
- 不自动修复所有几何问题，只提供可选的“保留最大连通块”修复。

## 接口设计

### 命令行

```bash
uv run python run.py \
  --model lamp \
  --expected-x 30 \
  --voxel-size 0.4 \
  --seed 42 \
  --out-dir result_agent_lamp
```

### 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--model` | str | 必填 | 模型名称，自动匹配 `auto_design/model/given_models/` 下的 `.stl` 与 `_joints.pkl` |
| `--expected-x` | float | 100 | 目标打印尺寸（毫米 mm），脚本内部转换为米后传给 auto_design |
| `--voxel-size` | float | 1.0 | 体素大小（毫米 mm） |
| `--seed` | int | 42 | 随机种子，保证可复现 |
| `--out-dir` | str | `result_agent_<model>` | 输出目录 |
| `--repair` | flag | False | 对断成多部分的连杆只保留最大连通块，保存为 `_repaired.stl` |
| `--skip-motors` | flag | False | 跳过电机可视化导出 |

### 模型匹配规则

1. 扫描 `auto_design/model/given_models/`。
2. 列出所有 `.stl` 文件（忽略大小写）。
3. 对给定的 `--model`，优先匹配名称前缀完全一致的 `.stl`，再尝试大小写不敏感匹配。
4. 对应 joints 文件为 `<model_stem>_joints.pkl`。
5. 若找不到或 joints 文件缺失，报错并打印可用模型列表。

## 输出目录结构

auto_design 本身会生成嵌套目录 `out-dir/<model>_<timestamp>/result_round1/`。`run.py` 运行结束后，会把最终产物整理成固定结构：

```
out-dir/
├── <model>_<timestamp>/     # auto_design 原始输出
│   └── result_round1/
│       ├── urdf/
│       │   ├── robot.urdf
│       │   ├── link_0.stl
│       │   └── ...
│       └── robot_result.pkl
├── parts/                    # 复制自 urdf/，米单位 STL + URDF
│   ├── robot.urdf
│   ├── link_0.stl
│   └── ...
├── parts_mm/                 # 缩放为毫米的 STL，可直接导入 OrcaSlicer
│   ├── link_0.stl
│   └── ...
├── motors/                   # 电机位置可视化（毫米单位）
│   ├── motor_link_0_father_0.stl
│   └── motors_combined.mm.stl
└── report.json               # 运行报告
```

## report.json 字段

```json
{
  "model": "lamp",
  "stl_path": "auto_design/model/given_models/lamp_scaled.stl",
  "joints_path": "auto_design/model/given_models/lamp_scaled_joints.pkl",
  "expected_x_mm": 30,
  "voxel_size_mm": 0.4,
  "seed": 42,
  "success": true,
  "exit_code": 0,
  "round": 1,
  "timings": {
    "design_seconds": 123.4,
    "export_mm_seconds": 0.5,
    "check_links_seconds": 0.3,
    "export_motors_seconds": 0.4
  },
  "files": {
    "urdf": "parts/robot.urdf",
    "parts": ["parts/link_0.stl", "..."],
    "parts_mm": ["parts_mm/link_0.stl", "..."],
    "motors": ["motors/motor_link_0_father_0.stl", "..."]
  },
  "link_checks": [
    {"file": "link_0.stl", "components": 1, "watertight": true},
    {"file": "link_1.stl", "components": 2, "watertight": false}
  ],
  "notes": []
}
```

## 组件与数据流

```
run.py
  │
  ├─ 模型解析 (resolve_model)
  │     └─ 匹配 .stl + _joints.pkl
  │
  ├─ 环境检查 (ensure_symlink)
  │     └─ 确保 anything2robot -> . 软链存在
  │
  ├─ auto_design (script.auto_design)
  │     └─ 输出到 out-dir/parts/
  │
  ├─ export_mm (复用 export_stl_to_mm 逻辑)
  │     └─ 输出到 out-dir/parts_mm/
  │
  ├─ check_links (复用 check_and_repair_links 逻辑)
  │     └─ 写入 report.json
  │
  └─ export_motors (复用 export_motor_visualization 逻辑)
        └─ 输出到 out-dir/motors/
```

## 实现策略

- `run.py` 直接 import `script.auto_design` 中的入口函数并构造参数，避免重复解析 auto_design 的 CLI。
- 将 `export_stl_to_mm.py`、`check_and_repair_links.py`、`export_motor_visualization.py` 的核心逻辑提炼为可 import 的函数，便于 `run.py` 调用；同时保留它们独立的 CLI 用法。
- 保持 `package://anything2robot` 不变，继续通过根目录软链 `anything2robot -> .` 解析。
- 不拆分 Qt UI；auto_design 内部通过 `save_only=True` 走无界面流程。

## 错误处理

- 模型解析失败：列出 `given_models/` 下所有可用 `.stl`，退出码 1。
- auto_design 失败：保存已产生的结果，`report.json` 中 `success=false`，`exit_code` 记录异常或非零值，仍尝试后续导出步骤（如果输出存在）。
- 连通性检查失败：记录到 `report.json`，若 `--repair` 则生成 `_repaired.stl`。
- 电机导出失败：除非 `--skip-motors`，否则记录错误但不中断整个流程。

## 测试计划

- 用 `lamp_scaled` 模型跑一次完整流程，验证 `report.json` 与输出目录结构。
- 用 `--model cactus` 跑一次，确认不同模型也能自动匹配。
- 用 `--repair` 跑一次，确认断开连杆会生成 `_repaired.stl`。
- 用 `--skip-motors` 跑一次，确认 motors 目录为空或不生成。

## 兼容性

- Python 3.11（与当前 `.python-version` 一致）。
- 依赖沿用 `pyproject.toml` / `uv.lock`，不新增包。
