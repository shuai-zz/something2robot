import importlib.util
from pathlib import Path

import numpy as np
import pytest
import trimesh


MODULE_PATH = Path(__file__).parents[1] / 'script' / 'attach_joint_library.py'
SPEC = importlib.util.spec_from_file_location('attach_joint_library', MODULE_PATH)
JOINTS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(JOINTS)


def test_hidden_ball_joint_is_watertight_and_connected():
    parent = trimesh.creation.box([24, 24, 20])
    parent.apply_translation([0, 0, 10])
    child = trimesh.creation.box([16, 16, 16])
    child.apply_translation([0, 0, -8])

    socket, stud, info = JOINTS.hidden_ball_joint(
        parent,
        child,
        parent_surface=np.array([0.0, 0.0, 0.0]),
        child_surface=np.array([0.0, 0.0, 0.0]),
        direction_into_child=np.array([0.0, 0.0, -1.0]),
        cfg={},
    )

    assert socket.is_watertight
    assert stud.is_watertight
    assert len(socket.split(only_watertight=False)) == 1
    assert len(stud.split(only_watertight=False)) == 1
    assert info['ball_center'] == [0.0, 0.0, 3.75]
    assert socket.volume < parent.volume
    assert stud.volume > child.volume


@pytest.mark.parametrize(
    'cfg',
    [
        {'ball_diameter': 0},
        {'ball_clearance': -0.1},
        {'neck_diameter': 0},
        {'socket_mouth_ratio': 1.0},
        {'socket_embed': 0},
    ],
)
def test_hidden_ball_joint_rejects_invalid_dimensions(cfg):
    box = trimesh.creation.box([20, 20, 20])
    with pytest.raises(ValueError):
        JOINTS.hidden_ball_joint(
            box.copy(),
            box.copy(),
            np.zeros(3),
            np.zeros(3),
            np.array([0.0, 0.0, -1.0]),
            cfg,
        )
