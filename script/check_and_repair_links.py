import os
import argparse
import trimesh


def check_urdf_folder_links(folder, repair=False, output_suffix='_repaired'):
    """检查 URDF 文件夹里每个 link STL 的连通性，返回结构化结果列表。"""
    stl_files = [f for f in os.listdir(folder) if f.endswith('.stl') and not f.endswith(output_suffix + '.stl')]
    results = []
    for f in sorted(stl_files):
        path = os.path.join(folder, f)
        mesh = trimesh.load(path)
        components = mesh.split(only_watertight=False)
        n = len(components)
        was_repaired = False

        if n > 1 and repair:
            # 保留最大连通块
            largest = max(components, key=lambda c: len(c.faces))
            out_path = os.path.join(folder, f.replace('.stl', output_suffix + '.stl'))
            largest.export(out_path)
            was_repaired = True

        results.append({
            'file': f,
            'components': n,
            'watertight': bool(mesh.is_watertight),
            'repaired': was_repaired,
        })
    return results


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
            path = os.path.join(folder, r['file'])
            mesh = trimesh.load(path)
            components = mesh.split(only_watertight=False)
            for i, c in enumerate(components):
                print(f"  部分 {i}: {len(c.vertices)} 顶点, {len(c.faces)} 面")

            if r['repaired']:
                out_path = os.path.join(folder, r['file'].replace('.stl', output_suffix + '.stl'))
                print(f"  -> 已修复并保存: {out_path}")

    print(f"\n总结: {len(broken)} 个文件断开" + ("，已修复" if repair and broken else ""))
    return results


def main():
    parser = argparse.ArgumentParser(description='检查并可选修复 auto_design 生成的 link STL 连通性。')
    parser.add_argument('--urdf_folder', type=str, required=True, help='包含 link STL 的 urdf 文件夹')
    parser.add_argument('--repair', action='store_true', help='如果断开，只保留最大连通块并另存为 *_repaired.stl')
    args = parser.parse_args()

    check_links(args.urdf_folder, repair=args.repair)


if __name__ == '__main__':
    main()
