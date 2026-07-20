import os
import argparse
import trimesh


def find_latest_urdf_folder(project_root='.'):
    """自动查找 result 目录下最新的 urdf 文件夹。"""
    result_dir = os.path.join(project_root, 'result')
    if not os.path.isdir(result_dir):
        raise FileNotFoundError(f"找不到 result 目录: {result_dir}")

    candidates = []
    for model_folder in os.listdir(result_dir):
        model_path = os.path.join(result_dir, model_folder)
        if not os.path.isdir(model_path):
            continue
        for round_name in os.listdir(model_path):
            round_path = os.path.join(model_path, round_name)
            if not os.path.isdir(round_path) or not round_name.startswith('result_round'):
                continue
            urdf_path = os.path.join(round_path, 'urdf')
            if os.path.isdir(urdf_path):
                candidates.append((os.path.getmtime(urdf_path), urdf_path))

    if not candidates:
        raise FileNotFoundError("在 result 目录下找不到任何 urdf 文件夹")

    candidates.sort(key=lambda x: x[0], reverse=True)
    return candidates[0][1]


def export_urdf_folder_to_mm(urdf_folder, output_folder, scale=1000.0):
    """Scale all STL files in a URDF folder to millimeters and export them.

    Args:
        urdf_folder: Path to the input URDF folder containing STL files.
        output_folder: Path to the output folder for scaled STL files.
        scale: Scaling factor to apply to each mesh. Defaults to 1000.0.

    Returns:
        A list of exported filenames.
    """
    os.makedirs(output_folder, exist_ok=True)
    exported_files = []

    for filename in sorted(os.listdir(urdf_folder)):
        if not filename.endswith('.stl'):
            continue
        src_path = os.path.join(urdf_folder, filename)
        mesh = trimesh.load(src_path)
        mesh.apply_scale(scale)

        dst_path = os.path.join(output_folder, filename)
        mesh.export(dst_path)
        print(f"  {filename:20s}  bounds {mesh.bounds.tolist()}")
        exported_files.append(filename)

    return exported_files


def main():
    parser = argparse.ArgumentParser(description='把 auto_design 生成的 STL 从米缩放到毫米，方便导入 OrcaSlicer 打印。')
    parser.add_argument('--urdf_folder', type=str, default=None,
                        help='输入 urdf 文件夹路径。默认自动查找 result 下最新的 urdf 文件夹。')
    parser.add_argument('--output_folder', type=str, default=None,
                        help='输出文件夹路径。默认在输入文件夹同级创建 *_mm 文件夹。')
    parser.add_argument('--scale', type=float, default=1000.0,
                        help='缩放倍数，默认 1000（米 -> 毫米）。')
    args = parser.parse_args()

    urdf_folder = args.urdf_folder or find_latest_urdf_folder()
    output_folder = args.output_folder or (urdf_folder + '_mm')

    print(f"输入: {urdf_folder}")
    print(f"输出: {output_folder}")

    exported_files = export_urdf_folder_to_mm(urdf_folder, output_folder, scale=args.scale)

    print(f"\n共导出 {len(exported_files)} 个 STL 到 {output_folder}")


if __name__ == '__main__':
    main()
