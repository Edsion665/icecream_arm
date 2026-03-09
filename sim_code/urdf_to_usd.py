#!/usr/bin/env python3
"""
将 URDF 导入并保存为 USD。
支持两种模型：ice_cream_0208.SLDASM（默认）、ice_cream_SINGLE.SLDASM。
用法：用 Isaac Sim 自带的 python.sh 运行（在项目根目录下执行）：
  /path/to/isaac-sim/python.sh sim_code/urdf_to_usd.py              # 默认 0208 -> ice_cream_arm.usd
  /path/to/isaac-sim/python.sh sim_code/urdf_to_usd.py --model SINGLE   # SINGLE -> ice_cream_single_arm.usd
 例如：~/isaac-sim/python.sh sim_code/urdf_to_usd.py --model SINGLE
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import argparse
import os
import sys

# 路径配置（默认 0208，可通过 --model 覆盖）
CODE_DIR = os.path.dirname(os.path.abspath(__file__))
ICECREAM_ROOT = os.path.dirname(CODE_DIR)

# 模型别名 -> (URDF 目录名, package 前缀, 输出 USD 文件名)
MODELS = {
    "0208": ("ice_cream_0208.SLDASM", "ice_cream_0208.SLDASM", "ice_cream_arm.usd"),
    "SINGLE": ("ice_cream_SINGLE.SLDASM", "ice_cream_SINGLE.SLDASM", "ice_cream_single_arm.usd"),
}


def get_paths(model_key: str):
    """根据模型键返回 URDF 目录、URDF 文件、输出 USD、package 前缀。"""
    dir_name, pkg_prefix, out_name = MODELS[model_key]
    urdf_dir = os.path.join(ICECREAM_ROOT, dir_name, "urdf")
    urdf_file = os.path.join(urdf_dir, f"{dir_name}.urdf")
    output_usd = os.path.join(CODE_DIR, out_name)
    return urdf_dir, urdf_file, output_usd, pkg_prefix


def prepare_urdf_with_resolved_meshes(urdf_file: str, urdf_dir: str, package_prefix: str):
    """将 URDF 中的 package:// 替换为相对路径 ../meshes/，便于 Isaac Sim 解析。"""
    with open(urdf_file, "r", encoding="utf-8") as f:
        content = f.read()
    content = content.replace(f"package://{package_prefix}/meshes/", "../meshes/")
    base = os.path.splitext(os.path.basename(urdf_file))[0]
    resolved_path = os.path.join(urdf_dir, f"{base}_resolved.urdf")
    with open(resolved_path, "w", encoding="utf-8") as f:
        f.write(content)
    return os.path.abspath(resolved_path)


def main():
    parser = argparse.ArgumentParser(description="URDF 导出为 USD")
    parser.add_argument(
        "--model",
        type=str,
        default="0208",
        choices=list(MODELS.keys()),
        help="模型：0208=ice_cream_0208.SLDASM（默认）, SINGLE=ice_cream_SINGLE.SLDASM",
    )
    args = parser.parse_args()

    urdf_dir, urdf_file, output_usd, package_prefix = get_paths(args.model)

    if not os.path.isfile(urdf_file):
        print(f"错误：未找到 URDF 文件 {urdf_file}")
        simulation_app.close()
        sys.exit(1)

    import omni.kit.commands

    resolved_urdf = prepare_urdf_with_resolved_meshes(urdf_file, urdf_dir, package_prefix)
    urdf_path = resolved_urdf

    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False
    import_config.fix_base = True
    import_config.make_default_prim = True
    import_config.create_physics_scene = True

    dest_path = os.path.abspath(output_usd)
    omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
        dest_path=dest_path,
    )

    print(f"已导出 USD: {dest_path}")
    print(f"模型: {args.model} | URDF: {urdf_file}")

    simulation_app.close()


if __name__ == "__main__":
    main()
