import os
import sys
import argparse
import shutil
import xml.etree.ElementTree as ET

project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(project_dir)
sys.path.append(os.path.join(project_dir, 'auto_design'))
sys.path.append(os.path.join(project_dir, 'auto_design/modules'))
sys.path.append(os.path.join(project_dir, 'script'))

from urdf_motor_adding import add_motor_to_urdf, fix_stl_path_issue


def remove_gazebo_tags(urdf_path, output_path):
    """urdfpy 保存时处理不了 <gazebo> 插件标签，先去掉。"""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    for gazebo in list(root.findall('gazebo')):
        root.remove(gazebo)
    tree.write(output_path, encoding='utf-8', xml_declaration=True)


def main():
    parser = argparse.ArgumentParser(description='在生成的 URDF 中插入电机模型（圆柱体），用于可视化电机位置。')
    parser.add_argument('--urdf_path', type=str, required=True, help='输入 URDF 文件路径')
    parser.add_argument('--pkl_path', type=str, required=True, help='对应的 robot_result.pkl 路径')
    parser.add_argument('--output_folder', type=str, default=None,
                        help='输出文件夹，默认在 urdf 文件夹同级创建 urdf_with_motors')
    args = parser.parse_args()

    if args.output_folder is None:
        # urdf_path 通常在 .../result_round1/urdf/xxx.urdf
        args.output_folder = os.path.join(os.path.dirname(os.path.dirname(args.urdf_path)), 'urdf_with_motors')

    os.makedirs(args.output_folder, exist_ok=True)

    # 先把原始 STL 复制到输出目录，这样 urdfpy 才能按文件名找到 mesh
    original_stl_folder = os.path.dirname(args.urdf_path)
    for f in os.listdir(original_stl_folder):
        if f.endswith('.stl'):
            shutil.copy(os.path.join(original_stl_folder, f), args.output_folder)

    # 把原始 URDF 的 mesh 路径修成纯文件名，避免 urdfpy 解析 package:// 出错
    fixed_urdf = os.path.join(args.output_folder, '_fixed_input.urdf')
    fix_stl_path_issue(args.urdf_path, fixed_urdf)

    # urdfpy 保存时处理不了 <gazebo> 插件标签，先去掉
    no_gazebo_urdf = os.path.join(args.output_folder, '_no_gazebo.urdf')
    remove_gazebo_tags(fixed_urdf, no_gazebo_urdf)

    # 插入电机
    add_motor_to_urdf(no_gazebo_urdf, args.pkl_path, args.output_folder)

    # 删除临时文件
    for temp in [fixed_urdf, no_gazebo_urdf]:
        if os.path.exists(temp):
            os.remove(temp)

    print(f"电机已加入，结果保存在: {args.output_folder}")


if __name__ == '__main__':
    main()
