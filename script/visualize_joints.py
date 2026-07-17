"""
Standalone joint-parameter visualization launcher.

Opens the project's LinkTreeGUI for an existing STL + _joints.pkl pair so you
can inspect / edit the annotated joints and axes without running the full
auto-design optimization pipeline.
"""
import argparse
import os
import sys

project_path = os.path.normpath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(project_path)
sys.path.append(os.path.normpath(os.path.join(project_path, 'auto_design')))
sys.path.append(os.path.normpath(os.path.join(project_path, 'auto_design/modules')))

from modules.mesh_loader import Custom_Mesh_Loader


def main():
    parser = argparse.ArgumentParser(description='Visualize existing joint parameters.')
    parser.add_argument('--model', type=str, required=True,
                        help='Model name prefix. Must match an .stl and '
                             '<name>_joints.pkl in auto_design/model/given_models/')
    parser.add_argument('--expected-x', type=float, default=50,
                        help='Expected X-axis length of the model (cm), only used '
                             'for scaling the displayed mesh.')
    args = parser.parse_args()

    model_dir = os.path.join(project_path, 'auto_design', 'model', 'given_models')
    mesh_path = os.path.normpath(os.path.join(model_dir, args.model + '.stl'))
    joint_path = os.path.normpath(os.path.join(model_dir, args.model + '_joints.pkl'))

    if not os.path.exists(mesh_path):
        raise FileNotFoundError(f'STL not found: {mesh_path}')
    if not os.path.exists(joint_path):
        raise FileNotFoundError(f'Joint pkl not found: {joint_path}')

    result_folder = os.path.normpath(os.path.join(project_path, 'result_visualize'))
    os.makedirs(result_folder, exist_ok=True)

    import types
    loader_args = types.SimpleNamespace(
        disable_joint_setting_ui=False,
        expected_x=args.expected_x,
        stl_mesh_path=mesh_path,
        joint_pkl_path=joint_path,
        model_name=args.model,
        result_folder=result_folder,
    )
    loader = Custom_Mesh_Loader(loader_args)
    loader.load_mesh(mesh_path)
    loader.load_joint_positions(joint_path)


if __name__ == '__main__':
    main()
