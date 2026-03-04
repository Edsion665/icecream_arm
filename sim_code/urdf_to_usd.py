#!/usr/bin/env python3
"""
第一套：将 URDF 导入并保存为 USD。
用法：在 Isaac Sim 的 Python 环境中运行，或使用 isaacsim -p 运行。
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import os
import sys

# 路径配置
CODE_DIR = os.path.dirname(os.path.abspath(__file__))
ICECREAM_ROOT = os.path.dirname(CODE_DIR)
URDF_DIR = os.path.join(ICECREAM_ROOT, "ice_cream_0208.SLDASM", "urdf")
URDF_FILE = os.path.join(URDF_DIR, "ice_cream_0208.SLDASM.urdf")
OUTPUT_USD = os.path.join(CODE_DIR, "ice_cream_arm.usd")


def prepare_urdf_with_resolved_meshes():
    """将 URDF 中的 package:// 替换为相对路径 ../meshes/，便于 Isaac Sim 解析。"""
    with open(URDF_FILE, "r", encoding="utf-8") as f:
        content = f.read()
    # 相对路径相对于 urdf 所在目录，../meshes/ 指向 ice_cream_1.SLDASM/meshes
    content = content.replace("package://ice_cream_0208.SLDASM/meshes/", "../meshes/")
    resolved_path = os.path.join(URDF_DIR, "ice_cream_0208_resolved.urdf")
    with open(resolved_path, "w", encoding="utf-8") as f:
        f.write(content)
    return os.path.abspath(resolved_path)


def main():
    import omni.kit.commands

    if not os.path.isfile(URDF_FILE):
        print(f"错误：未找到 URDF 文件 {URDF_FILE}")
        simulation_app.close()
        sys.exit(1)

    resolved_urdf = prepare_urdf_with_resolved_meshes()
    urdf_path = resolved_urdf

    # 创建新 stage 并导入 URDF 到目标 USD 文件
    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False
    import_config.fix_base = True
    import_config.make_default_prim = True
    import_config.create_physics_scene = True

    dest_path = os.path.abspath(OUTPUT_USD)
    omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
        dest_path=dest_path,
    )

    print(f"已导出 USD: {dest_path}")
    print("第一套完成：URDF -> USD 已保存。")

    simulation_app.close()


if __name__ == "__main__":
    main()
