# 项目整理设计（Git 仓库整理 + 安全清理）

## 日期
2026-07-17

## 背景
仓库当前处于"能跑但不干净"的状态：

- 核心管线文件全部未跟踪：`run.py`、`pyproject.toml`、`uv.lock`、`.python-version`、`AGENTS.md`、`script/` 下 5 个新脚本——fresh clone 无法运行管线。
- 6 个已跟踪文件有未提交修改（`README.md`、`auto_design/modules/` 4 个、`script/auto_design.py`，+66/-20 行）。
- `.gitignore` 有缺口：`result_*/`、`*.log`、`.DS_Store`、`.venv/`、`anything2robot` 自指 symlink 等均未覆盖；`**/docs/` 误伤 `docs/superpowers/` 设计文档。
- 80.4 MB 的 `.tools_openscad.AppImage` 被 git 跟踪（代码中无任何引用），是 248 MB git pack 的主因。
- 小垃圾：uv 模板残留 `main.py`、6 个根目录 `*.log`、`.bak`/`.backup` 文件、`__pycache__/`、`.pytest_cache/`、`.DS_Store`、若干空结果目录壳。

工作树总计 3.3 G，其中 `.venv` 1.9 G、`.worktrees` 398 M、`result*` 实验数据 208 M。

## 目标
- 让 `git status` 干净，fresh clone 即可获得完整可运行的管线。
- 提交范围：核心新文件 + 6 个已修改文件，全部提交（提交前 diff 摘要给用户过目）。
- 清理确定安全的小垃圾，释放少量空间、消除干扰。
- AppImage 移出 git 跟踪，文件保留在本地。

## 非目标
- 不动任何实验数据：`result*`（208 M）、`design/`（46 M）、`urdf/` 全部保持原样。
- 不合并、不删除 `.worktrees/web-joints`（`feat/web-joint-annotation-ui` 领先 main 19 个提交），仅在最终报告中说明状态。
- 不做 git 历史瘦身（filter-repo 清洗大二进制留作后续可选项）。
- 不删除上游遗留的已跟踪文件（`*.scad`、`fea_result.csv`、`env.yml`）和其他保留项（`script/backup/`、用户笔记 `cactus_run_comparison.md`），仅在报告中提示。
- 不做代码结构重组。

## 设计

### 1. `.gitignore` 修复
新增忽略规则：
```
*.log
.DS_Store
.superpowers/
anything2robot
result_*/
.venv/
.pytest_cache/
.tools_openscad.AppImage
```
收窄 `**/docs/` 规则，使 `docs/superpowers/` 可被跟踪。

### 2. 提交内容（分 2~3 个语义化 commit）
- commit A：`chore: fix .gitignore coverage`——仅 .gitignore 改动。
- commit B：`feat: add run.py pipeline entry and project config`——`run.py`、`pyproject.toml`、`uv.lock`、`.python-version`、`AGENTS.md`、`script/` 下 5 个未跟踪脚本（`add_motors_to_urdf.py`、`check_and_repair_links.py`、`export_motor_visualization.py`、`export_stl_to_mm.py`、`visualize_joints.py`）、`docs/superpowers/` 设计文档。
- commit C：6 个已修改文件（`README.md`、`auto_design/modules/{destruction_check,interference_removal,mesh_decomp,mesh_loader}.py`、`script/auto_design.py`）。提交前先把 diff 摘要给用户过目，commit message 根据 diff 实际内容撰写。

### 3. 删除清单（确定安全）
| 项 | 说明 |
|---|---|
| `main.py` | uv init 模板残留，未跟踪 |
| 根目录 6 个 `*.log` | 历史运行日志，未跟踪 |
| `script/motor_param_lib.py.bak` | 与现文件同尺寸的旧备份 |
| `auto_design/model/given_models/corgi_joints.pkl.backup` | 旧备份 |
| `__pycache__/`、`.pytest_cache/`、`.DS_Store` | 可再生缓存 |
| 空目录壳 | `result_visualize/`、`result_maneki_neko/` 内空子目录、`result/` 下 3 个空子目录 |

### 4. AppImage 处理
`git rm --cached .tools_openscad.AppImage` + 加入 `.gitignore`。文件保留在本地磁盘，pipeline 代码不引用它。

### 5. 验证
- `git status` 干净（剩余 untracked 均为刻意保留的实验数据/忽略项）。
- `git ls-files` 确认 `run.py`、`pyproject.toml` 等核心文件已入库。
- `uv run python run.py --help` 冒烟测试，确认清理未破坏管线。
- 最终报告：磁盘释放量、worktree 分支状态、遗留提示（上游旧文件、可选的历史瘦身）。
