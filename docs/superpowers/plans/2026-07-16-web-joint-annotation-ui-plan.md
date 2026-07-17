# Web-based Joint Annotation UI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a browser-based joint annotation tool (Three.js frontend + Flask backend) that produces the same `joints.pkl` as the existing Qt UI and can handle high-poly meshes.

**Architecture:** A Flask backend loads STL meshes via `trimesh`, simplifies them if needed, and exposes REST endpoints for mesh data and joint save/load. A static Three.js frontend renders the mesh, lets the user build the link tree, place joints, define axes, and validate annotations. Conversion between the frontend JSON and the existing `TreeNode`/`Link` pickle is handled by a dedicated converter module.

**Tech Stack:** Python 3.11, Flask (already present via Dash), trimesh, Three.js (via CDN), HTML/JS/CSS.

---

## File Structure

```
script/web_joint_annotation/
├── __init__.py
├── cli.py                 # Entry point: auto_design_web.py
├── server.py              # Flask app and REST endpoints
├── mesh_service.py        # STL loading, decimation, JSON export
├── joint_converter.py     # JSON <-> TreeNode/Link/joints.pkl
└── static/
    ├── index.html
    ├── app.js
    └── style.css

tests/web_joint_annotation/
├── __init__.py
├── test_joint_converter.py
└── test_server.py
```

---

## Task 1: Create Module Skeleton

**Files:**
- Create: `script/web_joint_annotation/__init__.py`
- Create: `script/web_joint_annotation/static/index.html`
- Create: `script/web_joint_annotation/static/app.js`
- Create: `script/web_joint_annotation/static/style.css`

- [ ] **Step 1: Create package init files**

`script/web_joint_annotation/__init__.py`:
```python
"""Web-based joint annotation UI for something2robot."""
```

`tests/web_joint_annotation/__init__.py`:
```python
"""Tests for web joint annotation."""
```

- [ ] **Step 2: Create static HTML skeleton**

`script/web_joint_annotation/static/index.html`:
```html
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Joint Annotation</title>
  <link rel="stylesheet" href="/static/style.css">
  <script src="https://unpkg.com/three@0.160.0/build/three.min.js"></script>
  <script src="https://unpkg.com/three@0.160.0/examples/js/controls/OrbitControls.js"></script>
</head>
<body>
  <div id="sidebar">
    <h2>Link Tree</h2>
    <div id="link-tree"></div>
    <div id="link-controls">
      <input type="text" id="new-link-name" placeholder="new link name">
      <select id="parent-link-select"></select>
      <button id="add-link-btn">Add Link</button>
    </div>
    <div id="joint-controls">
      <h3>Joints for <span id="current-link-name">BODY</span></h3>
      <input type="text" id="new-joint-name" placeholder="joint name">
      <button id="add-joint-mode-btn">Click on mesh to place</button>
      <ul id="joint-list"></ul>
    </div>
    <div id="axis-controls">
      <h3>Axis</h3>
      <input type="text" id="axis-origin" placeholder="origin x,y,z">
      <input type="text" id="axis-dir" placeholder="direction dx,dy,dz">
      <button id="set-axis-btn">Set Axis</button>
    </div>
    <div id="validation"></div>
    <button id="save-btn">Save Joints</button>
  </div>
  <div id="canvas-container"></div>
  <div id="status-bar"></div>
  <script src="/static/app.js"></script>
</body>
</html>
```

`script/web_joint_annotation/static/style.css`:
```css
* { box-sizing: border-box; margin: 0; padding: 0; }
body {
  display: flex;
  height: 100vh;
  font-family: system-ui, sans-serif;
  overflow: hidden;
}
#sidebar {
  width: 320px;
  min-width: 260px;
  padding: 12px;
  border-right: 1px solid #ccc;
  overflow-y: auto;
  display: flex;
  flex-direction: column;
  gap: 12px;
}
#canvas-container {
  flex: 1;
  position: relative;
}
#status-bar {
  position: fixed;
  bottom: 0;
  left: 320px;
  right: 0;
  padding: 6px 12px;
  background: #f5f5f5;
  border-top: 1px solid #ccc;
  font-size: 12px;
}
h2, h3 { font-size: 14px; margin-bottom: 6px; }
input, select, button {
  width: 100%;
  padding: 6px;
  margin-bottom: 6px;
  font-size: 12px;
}
button { cursor: pointer; }
button:disabled { opacity: 0.5; cursor: not-allowed; }
#link-tree { margin-bottom: 8px; }
.link-item {
  padding: 4px 6px;
  cursor: pointer;
  border-radius: 4px;
}
.link-item:hover { background: #eee; }
.link-item.selected { background: #cce5ff; }
#joint-list { list-style: none; }
#joint-list li {
  display: flex;
  justify-content: space-between;
  padding: 3px 0;
  font-size: 12px;
}
#error, .error { color: #d00; }
#warning, .warning { color: #a60; }
```

`script/web_joint_annotation/static/app.js` (initial stub):
```javascript
// Placeholder: full implementation in Task 5
console.log('Joint annotation UI loading...');
```

- [ ] **Step 3: Verify files exist**

Run:
```bash
ls -R script/web_joint_annotation
```

Expected: directories and files listed above.

---

## Task 2: Joint Converter (JSON <-> joints.pkl)

**Files:**
- Create: `script/web_joint_annotation/joint_converter.py`
- Test: `tests/web_joint_annotation/test_joint_converter.py`

- [ ] **Step 1: Write the failing test**

`tests/web_joint_annotation/test_joint_converter.py`:
```python
import os
import pickle
import sys
import pytest

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, PROJECT_ROOT)
sys.path.insert(0, os.path.join(PROJECT_ROOT, 'auto_design'))
sys.path.insert(0, os.path.join(PROJECT_ROOT, 'auto_design', 'modules'))

from web_joint_annotation.joint_converter import joints_to_json, json_to_joints, save_joints_pkl, load_joints_pkl


def test_round_trip_cactus():
    pkl_path = os.path.join(PROJECT_ROOT, 'auto_design', 'model', 'given_models', 'Cactus_Character_Scaled_joints.pkl')
    if not os.path.exists(pkl_path):
        pytest.skip('Cactus joint pkl not found')

    nodes = load_joints_pkl(pkl_path)
    data = joints_to_json(nodes)
    nodes2 = json_to_joints(data)

    assert set(nodes.keys()) == set(nodes2.keys())
    for name in nodes:
        link1 = nodes[name].val
        link2 = nodes2[name].val
        assert link1.name == link2.name
        assert list(link1.joints.keys()) == list(link2.joints.keys())
        for jn in link1.joints:
            assert pytest.approx(link1.joints[jn]) == list(link2.joints[jn])
        assert pytest.approx(link1.axis) == link2.axis
```

- [ ] **Step 2: Run test to verify it fails**

Run:
```bash
uv run pytest tests/web_joint_annotation/test_joint_converter.py -v
```

Expected: FAIL with `ModuleNotFoundError` or `ImportError` for `joint_converter`.

- [ ] **Step 3: Implement converter**

`script/web_joint_annotation/joint_converter.py`:
```python
"""Convert between frontend JSON and the existing joints.pkl TreeNode format."""
import os
import pickle
import sys
from typing import Any, Dict

import numpy as np

# Preserve existing import path expectations for the legacy code.
_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)
if os.path.join(_PROJECT_ROOT, 'auto_design') not in sys.path:
    sys.path.insert(0, os.path.join(_PROJECT_ROOT, 'auto_design'))
if os.path.join(_PROJECT_ROOT, 'auto_design', 'modules') not in sys.path:
    sys.path.insert(0, os.path.join(_PROJECT_ROOT, 'auto_design', 'modules'))

from data_struct import TreeNode
from mesh_loader import Link


def joints_to_json(nodes: Dict[str, TreeNode]) -> Dict[str, Any]:
    """Convert TreeNode dict to frontend JSON."""
    links = {}

    def walk(node):
        link = node.val
        children = [child.val.name for child in node.children]
        axis_data = {"origin": [0.0, 0.0, 0.0], "direction": [0.0, 0.0, 0.0]}
        if link.axis is not None and len(link.axis) >= 2:
            axis_data = {
                "origin": [float(v) for v in link.axis[0]],
                "direction": [float(v) for v in link.axis[1]],
            }

        parent_joint = None
        # Parent joint is the shared joint name that also exists in the parent link.
        # We infer it by looking at the parent node's children relation.
        # Since nodes dict is flat, we rebuild parent map first below.
        links[link.name] = {
            "joints": [
                {"name": name, "position": [float(v) for v in pos]}
                for name, pos in link.joints.items()
            ],
            "axis": axis_data,
            "children": children,
            "parent_joint": None,  # filled later
        }
        for child in node.children:
            walk(child)

    if "BODY" not in nodes:
        raise ValueError('Root link "BODY" not found')

    walk(nodes["BODY"])

    # Build parent map and shared joint names.
    parent_map = {}

    def build_parent_map(node):
        for child in node.children:
            parent_map[child.val.name] = node.val.name
            build_parent_map(child)

    build_parent_map(nodes["BODY"])

    for child_name, parent_name in parent_map.items():
        child_joints = {j["name"] for j in links[child_name]["joints"]}
        parent_joints = {j["name"] for j in links[parent_name]["joints"]}
        shared = child_joints & parent_joints
        if not shared:
            raise ValueError(f'No shared joint between {parent_name} and {child_name}')
        # Pick the first shared joint as the connection.
        links[child_name]["parent_joint"] = list(shared)[0]

    return {"version": "1.0", "links": links}


def json_to_joints(data: Dict[str, Any]) -> Dict[str, TreeNode]:
    """Convert frontend JSON to TreeNode dict."""
    if data.get("version") != "1.0":
        raise ValueError(f'Unsupported joints JSON version: {data.get("version")}')

    links_data = data["links"]
    if "BODY" not in links_data:
        raise ValueError('Root link "BODY" not found')

    # Build Link objects.
    link_objects = {}
    for name, ld in links_data.items():
        link = Link(name)
        for j in ld["joints"]:
            link.add_joint(j["name"], tuple(float(v) for v in j["position"]))
        axis = ld.get("axis")
        if axis:
            origin = axis.get("origin", [0.0, 0.0, 0.0])
            direction = axis.get("direction", [0.0, 0.0, 0.0])
            link.add_axis([*origin, *direction])
        link_objects[name] = link

    # Build tree.
    nodes = {}

    def build(name):
        if name in nodes:
            return nodes[name]
        link = link_objects[name]
        node = TreeNode(link)
        nodes[name] = node
        for child_name in links_data[name].get("children", []):
            child_node = build(child_name)
            node.add_child(child_node)
        return node

    build("BODY")
    return nodes


def load_joints_pkl(path: str) -> Dict[str, TreeNode]:
    with open(path, 'rb') as f:
        return pickle.load(f)


def save_joints_pkl(nodes: Dict[str, TreeNode], path: str) -> None:
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'wb') as f:
        pickle.dump(nodes, f)
```

- [ ] **Step 4: Run test to verify it passes**

Run:
```bash
uv run pytest tests/web_joint_annotation/test_joint_converter.py -v
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add script/web_joint_annotation/joint_converter.py tests/web_joint_annotation/test_joint_converter.py
git commit -m "feat(web-joints): add JSON <-> joints.pkl converter with round-trip test"
```

---

## Task 3: Mesh Service

**Files:**
- Create: `script/web_joint_annotation/mesh_service.py`
- Test: `tests/web_joint_annotation/test_mesh_service.py`

- [ ] **Step 1: Write the failing test**

`tests/web_joint_annotation/test_mesh_service.py`:
```python
import os
import sys
import pytest

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(PROJECT_ROOT, 'script'))

from web_joint_annotation.mesh_service import load_mesh_json


def test_load_cactus_mesh_json():
    stl_path = os.path.join(PROJECT_ROOT, 'auto_design', 'model', 'given_models', 'Cactus_Character_Scaled.stl')
    if not os.path.exists(stl_path):
        pytest.skip('Cactus STL not found')

    data = load_mesh_json(stl_path, max_faces=50000)
    assert "vertices" in data
    assert "faces" in data
    assert len(data["faces"]) <= 50000 * 3  # triangles as flat index list
    assert data["metadata"]["original_faces"] >= len(data["faces"]) // 3
```

- [ ] **Step 2: Run test to verify it fails**

Run:
```bash
uv run pytest tests/web_joint_annotation/test_mesh_service.py -v
```

Expected: FAIL with `ImportError` or `AttributeError`.

- [ ] **Step 3: Implement mesh service**

`script/web_joint_annotation/mesh_service.py`:
```python
"""Load and simplify STL meshes for the web annotator."""
import os
from typing import Any, Dict

import numpy as np
import trimesh


_DEFAULT_MAX_FACES = 100_000


def load_mesh_json(stl_path: str, max_faces: int = _DEFAULT_MAX_FACES) -> Dict[str, Any]:
    """Load an STL and return a JSON-friendly dict of vertices and triangle indices."""
    if not os.path.exists(stl_path):
        raise FileNotFoundError(f'STL not found: {stl_path}')

    mesh = trimesh.load_mesh(stl_path)
    mesh.merge_vertices()

    original_faces = len(mesh.faces)
    simplified = False
    target_faces = max_faces

    if original_faces > target_faces:
        # trimesh simplifies by vertex count; map to face count approximately.
        target_vertices = int(max(target_faces * 0.5, 1000))
        simplified_mesh = mesh.simplify_quadric_decimation(face_count=target_faces)
        if len(simplified_mesh.faces) < original_faces:
            mesh = simplified_mesh
            simplified = True

    vertices = np.asarray(mesh.vertices, dtype=np.float64)
    faces = np.asarray(mesh.faces, dtype=np.int32)

    return {
        "vertices": vertices.tolist(),
        "faces": faces.flatten().tolist(),
        "metadata": {
            "path": os.path.abspath(stl_path),
            "original_faces": int(original_faces),
            "current_faces": len(faces),
            "simplified": simplified,
        },
    }
```

- [ ] **Step 4: Run test to verify it passes**

Run:
```bash
uv run pytest tests/web_joint_annotation/test_mesh_service.py -v
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add script/web_joint_annotation/mesh_service.py tests/web_joint_annotation/test_mesh_service.py
git commit -m "feat(web-joints): add STL loading and decimation service"
```

---

## Task 4: Flask Server

**Files:**
- Create: `script/web_joint_annotation/server.py`
- Test: `tests/web_joint_annotation/test_server.py`

- [ ] **Step 1: Write the failing test**

`tests/web_joint_annotation/test_server.py`:
```python
import os
import sys
import pytest

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(PROJECT_ROOT, 'script'))

from web_joint_annotation.server import create_app


def test_index_and_mesh_endpoints():
    stl_path = os.path.join(PROJECT_ROOT, 'auto_design', 'model', 'given_models', 'Cactus_Character_Scaled.stl')
    if not os.path.exists(stl_path):
        pytest.skip('Cactus STL not found')

    out_pkl = os.path.join(PROJECT_ROOT, 'result_visualize', 'test_server_joints.pkl')
    app = create_app(stl_path, out_pkl)
    client = app.test_client()

    resp = client.get('/')
    assert resp.status_code == 200
    assert b'Joint Annotation' in resp.data

    resp = client.get('/api/mesh')
    assert resp.status_code == 200
    data = resp.get_json()
    assert 'vertices' in data
    assert 'faces' in data

    resp = client.post('/api/save', json={
        "version": "1.0",
        "links": {
            "BODY": {
                "joints": [
                    {"name": "foot", "position": [0.0, 0.0, 0.0]},
                    {"name": "waist", "position": [1.0, 0.0, 0.0]},
                ],
                "axis": {"origin": [0.0, 0.0, 0.0], "direction": [0.0, 0.0, 0.0]},
                "children": [],
                "parent_joint": None,
            }
        }
    })
    assert resp.status_code == 200
    assert os.path.exists(out_pkl)
```

- [ ] **Step 2: Run test to verify it fails**

Run:
```bash
uv run pytest tests/web_joint_annotation/test_server.py -v
```

Expected: FAIL with `ModuleNotFoundError` for `server`.

- [ ] **Step 3: Implement server**

`script/web_joint_annotation/server.py`:
```python
"""Flask server for the web joint annotation UI."""
import os
from typing import Any, Dict

from flask import Flask, jsonify, request, send_from_directory

from .joint_converter import json_to_joints, load_joints_pkl, save_joints_pkl, joints_to_json
from .mesh_service import load_mesh_json


def create_app(stl_path: str, joint_pkl_path: str, max_faces: int = 100_000) -> Flask:
    static_dir = os.path.join(os.path.dirname(__file__), 'static')
    app = Flask(__name__, static_folder=static_dir, static_url_path='/static')

    @app.route('/')
    def index():
        return send_from_directory(static_dir, 'index.html')

    @app.route('/api/mesh')
    def mesh():
        try:
            data = load_mesh_json(stl_path, max_faces=max_faces)
            return jsonify(data)
        except Exception as e:
            return jsonify({"error": str(e)}), 500

    @app.route('/api/joints')
    def joints():
        if not os.path.exists(joint_pkl_path):
            return jsonify({"version": "1.0", "links": {}})
        try:
            nodes = load_joints_pkl(joint_pkl_path)
            return jsonify(joints_to_json(nodes))
        except Exception as e:
            return jsonify({"error": str(e)}), 500

    @app.route('/api/save', methods=['POST'])
    def save():
        data: Dict[str, Any] = request.get_json(force=True)
        try:
            nodes = json_to_joints(data)
            # Validate basic rules before saving.
            errors = _validate(nodes)
            if errors:
                return jsonify({"success": False, "errors": errors}), 400
            save_joints_pkl(nodes, joint_pkl_path)
            return jsonify({"success": True, "path": joint_pkl_path})
        except Exception as e:
            return jsonify({"success": False, "error": str(e)}), 500

    return app


def _validate(nodes: Dict[str, Any]) -> list:
    errors = []
    if "BODY" not in nodes:
        errors.append('Root link "BODY" is required.')
        return errors

    foot_found = False
    for name, node in nodes.items():
        link = node.val
        if len(link.joints) < 2:
            errors.append(f'Link "{name}" must have at least 2 joints.')
        if link.axis is None or len(link.axis) < 2:
            if name != "BODY":
                errors.append(f'Link "{name}" must have a rotation axis.')
        for jn in link.joints:
            if 'foot' in jn:
                foot_found = True

    if not foot_found:
        errors.append('At least one joint name must contain "foot".')

    return errors
```

- [ ] **Step 4: Run test to verify it passes**

Run:
```bash
uv run pytest tests/web_joint_annotation/test_server.py -v
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add script/web_joint_annotation/server.py tests/web_joint_annotation/test_server.py
git commit -m "feat(web-joints): add Flask server with mesh/joints/save endpoints"
```

---

## Task 5: Frontend Implementation

**Files:**
- Modify: `script/web_joint_annotation/static/app.js`
- Modify: `script/web_joint_annotation/static/style.css` (minor additions)

- [ ] **Step 1: Implement core frontend logic**

`script/web_joint_annotation/static/app.js`:
```javascript
let scene, camera, renderer, controls, mesh, raycaster, pointer;
let annotationData = { version: "1.0", links: { BODY: {
  joints: [], axis: { origin: [0,0,0], direction: [0,0,0] }, children: [], parent_joint: null
}} };
let selectedLinkName = "BODY";
let placeJointMode = false;
let pendingJointName = "";

const jointSpheres = [];
const AXIS_HELPER_LENGTH = 2.0;

async function init() {
  const container = document.getElementById('canvas-container');

  scene = new THREE.Scene();
  scene.background = new THREE.Color(0xf0f0f0);

  camera = new THREE.PerspectiveCamera(50, container.clientWidth / container.clientHeight, 0.01, 1000);
  camera.position.set(0, 0, 20);

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(container.clientWidth, container.clientHeight);
  container.appendChild(renderer.domElement);

  controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;

  const ambient = new THREE.AmbientLight(0xffffff, 0.6);
  scene.add(ambient);
  const directional = new THREE.DirectionalLight(0xffffff, 0.5);
  directional.position.set(1, 1, 1);
  scene.add(directional);

  raycaster = new THREE.Raycaster();
  pointer = new THREE.Vector2();

  renderer.domElement.addEventListener('pointerdown', onPointerDown);
  window.addEventListener('resize', onWindowResize);

  await loadMesh();
  await loadJoints();
  renderLinkTree();
  updateJointList();
  animate();
}

async function loadMesh() {
  const resp = await fetch('/api/mesh');
  const data = await resp.json();
  if (data.error) {
    setStatus(data.error, 'error');
    return;
  }

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(data.vertices.flat(), 3));
  geometry.setIndex(data.faces);
  geometry.computeVertexNormals();

  const material = new THREE.MeshStandardMaterial({
    color: 0x888888,
    roughness: 0.7,
    metalness: 0.1,
    side: THREE.DoubleSide,
  });
  mesh = new THREE.Mesh(geometry, material);
  scene.add(mesh);

  // Center camera on mesh.
  geometry.computeBoundingSphere();
  controls.target.copy(geometry.boundingSphere.center);
  camera.position.copy(geometry.boundingSphere.center).add(new THREE.Vector3(0, 0, geometry.boundingSphere.radius * 3));
  controls.update();

  setStatus(`Loaded ${data.metadata.current_faces} faces (original ${data.metadata.original_faces})`);
}

async function loadJoints() {
  const resp = await fetch('/api/joints');
  const data = await resp.json();
  if (data.error) {
    setStatus(data.error, 'error');
    return;
  }
  if (Object.keys(data.links || {}).length > 0) {
    annotationData = data;
  }
}

function renderLinkTree() {
  const container = document.getElementById('link-tree');
  container.innerHTML = '';

  function render(name, depth) {
    const div = document.createElement('div');
    div.className = 'link-item' + (name === selectedLinkName ? ' selected' : '');
    div.style.paddingLeft = (depth * 12 + 6) + 'px';
    div.textContent = name;
    div.onclick = () => selectLink(name);
    container.appendChild(div);
    const link = annotationData.links[name];
    if (link && link.children) {
      link.children.forEach(child => render(child, depth + 1));
    }
  }
  render('BODY', 0);

  const parentSelect = document.getElementById('parent-link-select');
  parentSelect.innerHTML = '';
  Object.keys(annotationData.links).forEach(name => {
    const opt = document.createElement('option');
    opt.value = name;
    opt.textContent = name;
    parentSelect.appendChild(opt);
  });
}

function selectLink(name) {
  selectedLinkName = name;
  document.getElementById('current-link-name').textContent = name;
  renderLinkTree();
  updateJointList();
  drawJointsAndAxes();
}

function updateJointList() {
  const list = document.getElementById('joint-list');
  list.innerHTML = '';
  const link = annotationData.links[selectedLinkName];
  if (!link) return;
  link.joints.forEach(j => {
    const li = document.createElement('li');
    li.innerHTML = `<span>${j.name}: (${j.position.map(v => v.toFixed(2)).join(', ')})</span>`;
    const del = document.createElement('button');
    del.textContent = '×';
    del.onclick = () => removeJoint(j.name);
    li.appendChild(del);
    list.appendChild(li);
  });
}

document.getElementById('add-link-btn').onclick = () => {
  const name = document.getElementById('new-link-name').value.trim();
  const parent = document.getElementById('parent-link-select').value;
  if (!name || annotationData.links[name]) {
    setStatus('Invalid or duplicate link name', 'error');
    return;
  }
  annotationData.links[name] = {
    joints: [], axis: { origin: [0,0,0], direction: [0,0,0] }, children: [], parent_joint: null
  };
  annotationData.links[parent].children.push(name);
  document.getElementById('new-link-name').value = '';
  renderLinkTree();
  selectLink(name);
};

document.getElementById('add-joint-mode-btn').onclick = () => {
  const name = document.getElementById('new-joint-name').value.trim();
  if (!name) {
    setStatus('Enter a joint name first', 'error');
    return;
  }
  pendingJointName = name;
  placeJointMode = true;
  setStatus('Click on the mesh to place joint: ' + name);
};

function onPointerDown(event) {
  if (!placeJointMode || !mesh) return;
  const rect = renderer.domElement.getBoundingClientRect();
  pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(pointer, camera);
  const intersects = raycaster.intersectObject(mesh);
  if (intersects.length === 0) return;

  const point = intersects[0].point;
  addJoint(pendingJointName, [point.x, point.y, point.z]);
  placeJointMode = false;
}

function addJoint(name, position) {
  const link = annotationData.links[selectedLinkName];
  const existing = link.joints.find(j => j.name === name);
  if (existing) {
    existing.position = position;
  } else {
    link.joints.push({ name, position });
  }

  // If this link has a parent and the shared parent joint matches, sync position.
  if (link.parent_joint === name) {
    const parentName = findParent(selectedLinkName);
    if (parentName) {
      const parent = annotationData.links[parentName];
      const pj = parent.joints.find(j => j.name === name);
      if (pj) pj.position = position.slice();
    }
  }
  updateJointList();
  drawJointsAndAxes();
  validate();
}

function removeJoint(name) {
  const link = annotationData.links[selectedLinkName];
  link.joints = link.joints.filter(j => j.name !== name);
  updateJointList();
  drawJointsAndAxes();
  validate();
}

function findParent(childName) {
  for (const [name, link] of Object.entries(annotationData.links)) {
    if (link.children && link.children.includes(childName)) return name;
  }
  return null;
}

document.getElementById('set-axis-btn').onclick = () => {
  const link = annotationData.links[selectedLinkName];
  const origin = document.getElementById('axis-origin').value.split(',').map(parseFloat);
  const dir = document.getElementById('axis-dir').value.split(',').map(parseFloat);
  if (origin.length !== 3 || dir.length !== 3 || origin.some(isNaN) || dir.some(isNaN)) {
    setStatus('Axis must be three comma-separated numbers for origin and direction', 'error');
    return;
  }
  link.axis = { origin, direction: dir };
  drawJointsAndAxes();
  validate();
};

function drawJointsAndAxes() {
  jointSpheres.forEach(s => scene.remove(s));
  jointSpheres.length = 0;

  Object.entries(annotationData.links).forEach(([name, link]) => {
    const color = name === selectedLinkName ? 0x00aaff : 0xff6600;
    link.joints.forEach(j => {
      const geo = new THREE.SphereGeometry(0.15, 16, 16);
      const mat = new THREE.MeshBasicMaterial({ color });
      const sphere = new THREE.Mesh(geo, mat);
      sphere.position.set(...j.position);
      scene.add(sphere);
      jointSpheres.push(sphere);
    });

    if (name !== 'BODY' && link.axis) {
      const origin = new THREE.Vector3(...link.axis.origin);
      const dir = new THREE.Vector3(...link.axis.direction).normalize();
      const end = origin.clone().add(dir.multiplyScalar(AXIS_HELPER_LENGTH));
      const lineGeo = new THREE.BufferGeometry().setFromPoints([origin, end]);
      const lineMat = new THREE.LineBasicMaterial({ color: 0x00ff00 });
      const line = new THREE.Line(lineGeo, lineMat);
      scene.add(line);
      jointSpheres.push(line);
    }
  });
}

function validate() {
  const errors = [];
  if (!annotationData.links.BODY) errors.push('BODY link is required.');

  let footFound = false;
  Object.entries(annotationData.links).forEach(([name, link]) => {
    if (link.joints.length < 2) errors.push(`Link "${name}" needs at least 2 joints.`);
    if (name !== 'BODY' && (!link.axis || (link.axis.direction[0] === 0 && link.axis.direction[1] === 0 && link.axis.direction[2] === 0))) {
      errors.push(`Link "${name}" needs a non-zero rotation axis.`);
    }
    link.joints.forEach(j => { if (j.name.includes('foot')) footFound = true; });
  });
  if (!footFound) errors.push('At least one joint name must contain "foot".');

  const box = document.getElementById('validation');
  if (errors.length) {
    box.innerHTML = errors.map(e => `<div class="error">${e}</div>`).join('');
    document.getElementById('save-btn').disabled = true;
  } else {
    box.innerHTML = '<div style="color:green">Ready to save</div>';
    document.getElementById('save-btn').disabled = false;
  }
}

document.getElementById('save-btn').onclick = async () => {
  const resp = await fetch('/api/save', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(annotationData)
  });
  const result = await resp.json();
  if (result.success) {
    setStatus('Saved to ' + result.path);
  } else {
    setStatus('Save failed: ' + (result.error || result.errors?.join('; ')), 'error');
  }
};

function setStatus(msg, type = '') {
  const bar = document.getElementById('status-bar');
  bar.textContent = msg;
  bar.className = type;
}

function onWindowResize() {
  const container = document.getElementById('canvas-container');
  camera.aspect = container.clientWidth / container.clientHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(container.clientWidth, container.clientHeight);
}

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}

init();
```

- [ ] **Step 2: Verify the frontend loads without errors**

Start the server (Task 6 CLI) and open `http://127.0.0.1:8050/`. Check browser console for JS errors.

- [ ] **Step 3: Commit**

```bash
git add script/web_joint_annotation/static/app.js script/web_joint_annotation/static/style.css script/web_joint_annotation/static/index.html
git commit -m "feat(web-joints): implement Three.js annotation frontend"
```

---

## Task 6: CLI Entry Point

**Files:**
- Create: `script/auto_design_web.py`

- [ ] **Step 1: Implement CLI**

`script/auto_design_web.py`:
```python
#!/usr/bin/env python3
"""Launch the web-based joint annotation UI."""
import argparse
import os
import sys
import webbrowser

project_path = os.path.normpath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, project_path)
sys.path.insert(0, os.path.normpath(os.path.join(project_path, 'script')))

from web_joint_annotation.server import create_app


def main():
    parser = argparse.ArgumentParser(description='Web-based joint annotation UI')
    parser.add_argument('--stl', type=str, required=True,
                        help='Path to the STL mesh file')
    parser.add_argument('--joint_pkl', type=str, required=True,
                        help='Path where the joints pickle will be written')
    parser.add_argument('--port', type=int, default=8050,
                        help='Port for the local web server')
    parser.add_argument('--max-faces', type=int, default=100_000,
                        help='Target maximum number of faces after simplification')
    parser.add_argument('--no-browser', action='store_true',
                        help='Do not open the browser automatically')
    args = parser.parse_args()

    if not os.path.exists(args.stl):
        raise FileNotFoundError(f'STL not found: {args.stl}')

    app = create_app(args.stl, args.joint_pkl, max_faces=args.max_faces)
    url = f'http://127.0.0.1:{args.port}'
    print(f'Starting joint annotation UI at {url}')
    if not args.no_browser:
        webbrowser.open(url)
    app.run(host='127.0.0.1', port=args.port, debug=False)


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Run a quick smoke test**

Run:
```bash
uv run python script/auto_design_web.py \
  --stl auto_design/model/given_models/Cactus_Character_Scaled.stl \
  --joint_pkl result_visualize/cactus_web_joints.pkl \
  --no-browser
```

Then in another terminal:
```bash
curl -s http://127.0.0.1:8050/api/mesh | head -c 200
```

Expected: JSON beginning with `{"vertices":...`.

Stop the server with Ctrl-C.

- [ ] **Step 3: Commit**

```bash
git add script/auto_design_web.py
git commit -m "feat(web-joints): add CLI entry point auto_design_web.py"
```

---

## Task 7: Integration Test

**Files:**
- Test: `tests/web_joint_annotation/test_integration.py`

- [ ] **Step 1: Write integration test**

`tests/web_joint_annotation/test_integration.py`:
```python
import os
import sys
import subprocess
import time

import pytest

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))


def test_cactus_annotation_round_trip():
    stl = os.path.join(PROJECT_ROOT, 'auto_design', 'model', 'given_models', 'Cactus_Character_Scaled.stl')
    pkl = os.path.join(PROJECT_ROOT, 'result_visualize', 'cactus_web_test.pkl')
    if not os.path.exists(stl):
        pytest.skip('Cactus STL not found')

    # Ensure output dir exists.
    os.makedirs(os.path.dirname(pkl), exist_ok=True)
    if os.path.exists(pkl):
        os.remove(pkl)

    # Start server in background.
    proc = subprocess.Popen(
        ['uv', 'run', 'python', 'script/auto_design_web.py', '--stl', stl, '--joint_pkl', pkl, '--port', '18050', '--no-browser'],
        cwd=PROJECT_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    time.sleep(3)
    try:
        import urllib.request
        import json

        with urllib.request.urlopen('http://127.0.0.1:18050/api/mesh', timeout=10) as resp:
            data = json.loads(resp.read().decode())
        assert 'vertices' in data
        assert 'faces' in data

        # Save a minimal valid annotation.
        req = urllib.request.Request(
            'http://127.0.0.1:18050/api/save',
            data=json.dumps({
                "version": "1.0",
                "links": {
                    "BODY": {
                        "joints": [
                            {"name": "foot", "position": [0.0, 0.0, 0.0]},
                            {"name": "waist", "position": [1.0, 0.0, 0.0]},
                        ],
                        "axis": {"origin": [0,0,0], "direction": [0,0,0]},
                        "children": [],
                        "parent_joint": None,
                    }
                }
            }).encode(),
            headers={'Content-Type': 'application/json'},
            method='POST'
        )
        with urllib.request.urlopen(req, timeout=10) as resp:
            result = json.loads(resp.read().decode())
        assert result["success"] is True
        assert os.path.exists(pkl)
    finally:
        proc.terminate()
        proc.wait(timeout=5)
```

- [ ] **Step 2: Run integration test**

Run:
```bash
uv run pytest tests/web_joint_annotation/test_integration.py -v
```

Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add tests/web_joint_annotation/test_integration.py
git commit -m "test(web-joints): add server integration test"
```

---

## Task 8: End-to-End Validation with Existing Pipeline

- [ ] **Step 1: Generate joints.pkl for cactus using the web UI**

Run:
```bash
uv run python script/auto_design_web.py \
  --stl auto_design/model/given_models/Cactus_Character_Scaled.stl \
  --joint_pkl auto_design/model/given_models/Cactus_Character_Scaled_joints_web.pkl
```

In the browser, annotate a minimal valid tree (BODY with foot+waist, etc.) and save.

- [ ] **Step 2: Run the full pipeline with the new joints file**

Rename or symlink so `run.py` picks it up, or copy over the original:
```bash
cp auto_design/model/given_models/Cactus_Character_Scaled_joints_web.pkl \
   auto_design/model/given_models/Cactus_Character_Scaled_joints.pkl
uv run python run.py --model Cactus_Character_Scaled --expected-x 100 --voxel-size 1.0 --seed 42 --out-dir result_web_cactus
```

Expected: pipeline completes without errors related to joints format.

- [ ] **Step 3: Document any deviations**

If the pipeline fails due to axis semantics or shared-joint rules, update `joint_converter.py` and `server.py` validation, then re-run.

- [ ] **Step 4: Final commit**

```bash
git add script/web_joint_annotation/ tests/web_joint_annotation/ script/auto_design_web.py
git commit -m "feat: web-based joint annotation UI MVP"
```

---

## Self-Review Checklist

- [ ] **Spec coverage:** Every section of the design doc maps to at least one task above.
- [ ] **Placeholder scan:** No TODOs, TBDs, or vague instructions remain.
- [ ] **Type consistency:** JSON field names (`links`, `joints`, `axis`, `children`, `parent_joint`) match between frontend, backend converter, and tests.
- [ ] **Path consistency:** All file paths use project-root-relative or absolute derivation; CLI matches spec.

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-07-16-web-joint-annotation-ui-plan.md`.

Two execution options:

1. **Subagent-Driven (recommended)** - Dispatch a fresh subagent per task, review between tasks, fast iteration.
2. **Inline Execution** - Execute tasks in this session using executing-plans, batch execution with checkpoints.

Which approach?
