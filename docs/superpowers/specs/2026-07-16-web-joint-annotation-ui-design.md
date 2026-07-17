# Web-based Joint Annotation UI Design

## Objective

Replace the current Qt + PyVista/Dash joint annotation UI in `something2robot` with a browser-based 3D annotation tool that can smoothly handle high-poly models (e.g., the Pikachu STL with ~1.5M faces) while producing the same `joints.pkl` output required by the pipeline.

## Background

The existing UI in `auto_design/modules/mesh_loader.py` uses Qt + PyVistaQt for 3D rendering and Dash/Plotly for a secondary web view. On dense meshes it becomes unusably slow. The goal is to keep the annotation semantics unchanged but move rendering and interaction to a modern frontend 3D engine.

## Scope

- **In scope (MVP)**
  - Load an STL from `auto_design/model/given_models/`
  - Simplify dense meshes automatically so the browser stays interactive
  - Build the link tree with a fixed `BODY` root
  - Place joints by clicking on the mesh surface
  - Define rotation axes for non-root links
  - Validate annotations against existing rules
  - Export to the existing `joints.pkl` format
- **Out of scope (MVP)**
  - Undo/redo (users delete and re-place for now)
  - Full mesh editing (cutting, boolean operations, etc.)
  - Real-time collaborative editing
  - Replacing the entire `auto_design.py` CLI; this tool only replaces the annotation step

## Decisions Made

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Scope | C: Web Viewer + Annotation tool, parallel to existing UI | Does not break existing pipeline; can be swapped in later |
| Data format compatibility | C: Decide later; frontend uses JSON, backend converts to `joints.pkl` | Keeps frontend simple and decoupled from Python pickle internals |
| 3D engine | Three.js | Largest ecosystem, lowest startup cost for click-to-place annotation |
| Architecture | A: Standalone web tool | Best performance, cleanest separation, easiest to iterate |
| MVP feature set | A: Full annotation workflow | Direct replacement for current UI from day one |
| Undo/redo | Deferred | Delete-and-replace is acceptable for MVP |

## Architecture

```
┌─────────────────┐      HTTP/WebSocket       ┌─────────────────┐
│   Browser       │  <--------------------->  │  Python backend │
│  (Three.js)     │                           │  (FastAPI/Flask)│
└─────────────────┘                           └─────────────────┘
         │                                            │
         │ 1. Load mesh                               │ 1. Read STL with trimesh
         │ 2. Place joints / build tree               │ 2. Decimate if needed
         │ 3. Define axes                             │ 3. Convert to glTF/JSON
         │ 4. POST annotation JSON                    │ 4. Validate + convert to TreeNode
         │                                            │ 5. Write joints.pkl
```

### Components

1. **Backend server** (`script/auto_design_web.py`)
   - Loads STL via `trimesh`
   - Simplifies meshes above a face threshold (default 100k target 50k)
   - Serves mesh geometry and any existing joints
   - Receives annotation JSON, validates it, converts to `TreeNode`, writes `joints.pkl`

2. **Frontend app**
   - Three.js scene with orbit controls
   - Left sidebar for link tree and property editing
   - Click-to-place joint interaction with surface snapping
   - Inline validation feedback

3. **Converter module**
   - `json_to_joint_pkl()` / `joint_pkl_to_json()`
   - Ensures round-trip compatibility with existing cactus/lamp joints

## Data Flow

1. User runs:
   ```bash
   uv run python script/auto_design_web.py \
     --stl auto_design/model/given_models/cactus.stl \
     --joint_pkl auto_design/model/given_models/cactus_joints.pkl
   ```
2. Backend loads STL, decimates if needed, starts server, opens browser.
3. Frontend requests `/api/mesh` and `/api/joints`.
4. User annotates in browser; state lives in the frontend.
5. On save, frontend POSTs `/api/save` with JSON payload.
6. Backend validates, converts to `TreeNode`, writes `joints.pkl`.

## Frontend JSON Format

```json
{
  "version": "1.0",
  "links": {
    "BODY": {
      "joints": [
        {"name": "j_body_left_arm", "position": [x, y, z]}
      ],
      "axis": {"origin": [x, y, z], "direction": [0, 0, 0]},
      "children": ["left_arm"]
    },
    "left_arm": {
      "parent_joint": "j_body_left_arm",
      "joints": [
        {"name": "j_left_arm_foot", "position": [x, y, z]}
      ],
      "axis": {"origin": [x, y, z], "direction": [dx, dy, dz]},
      "children": []
    }
  }
}
```

- `parent_joint` links a child to its parent. The referenced joint must exist in the parent link with the same name and coordinates.
- Root link (`BODY`) uses axis direction `[0, 0, 0]`.
- Ground-contact joints should contain `foot` in their names.

## UI Workflow

1. **Load model** — progress bar while the mesh is simplified and transferred.
2. **Link tree panel** — fixed `BODY` root; add/delete/rename/drag child links.
3. **Place joints** — select a link, click mesh surface; joints snap to the nearest triangle and are visualized as spheres.
4. **Define axis** — for non-root links, pick two points (axis origin + direction) or type a vector.
5. **Validate** — inline checks:
   - Each link has at least 2 joints
   - Shared parent/child joints match in name and position
   - Non-root links have a non-zero axis
   - Ground joints are named with `foot`
6. **Save** — disabled until validation passes; writes `joints.pkl` via backend.

## Error Handling

| Scenario | Behavior |
|----------|----------|
| Mesh > face threshold | Auto-decimate; show warning banner with before/after face counts |
| Invalid annotation state | Save disabled; invalid fields highlighted with tooltip |
| Click misses mesh | Ignore or snap to nearest vertex |
| Backend conversion fails | Show error modal; log full traceback to terminal |
| Browser unsupported | Require a modern browser; show message if WebGL unavailable |

## Testing

- **Backend unit tests**: JSON ↔ `joints.pkl` round-trip using existing cactus and lamp pickles.
- **Smoke test**: Start server, load STL, place one joint, save, assert file exists.
- **Integration test**: Annotate cactus with the new tool, then run `run.py` to verify the downstream pipeline still works.

## Open Questions / Future Work

- Exact decimation threshold and target face count should be tuned after testing with Pikachu/cactus.
- Whether to support loading existing joints for editing (planned but needs round-trip verification).
- Undo/redo, keyboard shortcuts, and multi-viewport can be added once the core workflow is solid.
