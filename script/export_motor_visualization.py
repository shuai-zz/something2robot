import os
import sys
import argparse
import pickle
import numpy as np
import trimesh

project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(project_dir)
sys.path.append(os.path.join(project_dir, 'auto_design'))
sys.path.append(os.path.join(project_dir, 'auto_design/modules'))
sys.path.append(os.path.join(project_dir, 'script'))

from motor_param_lib import MotorParameterLib
from interference_removal import RobotOptResult

motor_lib = MotorParameterLib().get_motor_lib()


def rotation_matrix_from_vectors(vec1, vec2):
    """Return a rotation matrix that aligns vec1 to vec2."""
    a = vec1 / np.linalg.norm(vec1)
    b = vec2 / np.linalg.norm(vec2)
    cross = np.cross(a, b)
    dot = np.dot(a, b)
    if np.isclose(dot, 1.0):
        return np.eye(3)
    elif np.isclose(dot, -1.0):
        perp = np.array([1, 0, 0]) if not np.allclose(a, [1, 0, 0]) else np.array([0, 1, 0])
        axis = np.cross(a, perp)
        axis = axis / np.linalg.norm(axis)
        return trimesh.transformations.rotation_matrix(np.pi, axis)[:3, :3]
    skew = np.array([
        [0, -cross[2], cross[1]],
        [cross[2], 0, -cross[0]],
        [-cross[1], cross[0], 0]
    ])
    return np.eye(3) + skew + skew @ skew * ((1 - dot) / (np.linalg.norm(cross) ** 2))


def export_motors_from_pkl(pkl_path, output_folder=None, unit='mm'):
    """Export motor visualization STLs from a robot_result.pkl file.

    Args:
        pkl_path: Path to robot_result.pkl.
        output_folder: Folder to save STL files. Defaults to a 'motors' folder
            next to the pkl file.
        unit: Output unit, either 'm' or 'mm'.

    Returns:
        List of exported filenames (including the combined file, if any).
    """
    if output_folder is None:
        output_folder = os.path.join(os.path.dirname(pkl_path), 'motors')
    os.makedirs(output_folder, exist_ok=True)

    scale = 1000.0 if unit == 'mm' else 1.0
    cm_to_out = 10.0 if unit == 'mm' else 0.01

    rr = pickle.load(open(pkl_path, 'rb'))

    combined = []
    motor_idx_counter = {}
    exported_files = []

    for link_name, link_result in rr.link_dict.items():
        for i, tenon_type in enumerate(link_result.tenon_type):
            # 有些 link 会有 father 和 child 两个 tenon，这里都导出
            tenon_pos = np.array(link_result.tenon_pos[i][:3])
            tenon_dir = np.array(link_result.tenon_pos[i][3:6])
            tenon_dir = tenon_dir / np.linalg.norm(tenon_dir)
            tenon_idx = link_result.tenon_idx[i]

            motor_height_cm, motor_radius_cm, _ = motor_lib[tenon_idx]
            radius = motor_radius_cm * cm_to_out
            height = motor_height_cm * cm_to_out

            # 电机中心沿 tenon_dir 偏移 half height
            center = tenon_pos * scale + height * 0.5 * tenon_dir

            cylinder = trimesh.creation.cylinder(radius=radius, height=height, sections=32)
            rot = rotation_matrix_from_vectors(np.array([0, 0, 1]), tenon_dir)
            transform = np.eye(4)
            transform[:3, :3] = rot
            transform[:3, 3] = center
            cylinder.apply_transform(transform)

            # 给不同电机类型上不同颜色
            colors = [[255, 0, 0], [0, 255, 0], [0, 0, 255], [255, 255, 0], [0, 255, 255]]
            color = colors[tenon_idx % len(colors)]
            cylinder.visual.vertex_colors = color

            count = motor_idx_counter.get((link_name, tenon_idx), 0)
            filename = f"motor_{link_name}_{tenon_type}_{count}.stl"
            motor_idx_counter[(link_name, tenon_idx)] = count + 1
            cylinder.export(os.path.join(output_folder, filename))
            combined.append(cylinder)
            exported_files.append(filename)
            print(f"导出: {filename} | 电机类型索引={tenon_idx} | 半径={radius:.2f} {unit} | 高度={height:.2f} {unit}")

    if combined:
        combined_filename = f"motors_combined.{unit.lower()}.stl"
        scene = trimesh.util.concatenate(combined)
        scene.export(os.path.join(output_folder, combined_filename))
        exported_files.append(combined_filename)
        print(f"\n合并电机模型已保存: {os.path.join(output_folder, combined_filename)}")

    return exported_files


def main():
    parser = argparse.ArgumentParser(description='从 robot_result.pkl 导出电机可视化 STL，用于查看电机应该插在哪里。')
    parser.add_argument('--pkl_path', type=str, required=True, help='robot_result.pkl 路径')
    parser.add_argument('--output_folder', type=str, default=None, help='输出文件夹')
    parser.add_argument('--unit', type=str, default='mm', choices=['m', 'mm'], help='输出单位')
    args = parser.parse_args()

    export_motors_from_pkl(args.pkl_path, args.output_folder, args.unit)
    output_folder = args.output_folder if args.output_folder is not None else \
        os.path.join(os.path.dirname(args.pkl_path), 'motors')
    print(f"\n所有电机 STL 已保存到: {output_folder}")


if __name__ == '__main__':
    main()
