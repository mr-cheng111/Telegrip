"""
测试 telegrip 当前 Mink 运动学链路集成。
"""

import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from telegrip.config import TelegripConfig
from telegrip.core.robot_interface import RobotInterface


def main():
    print("=== Mink Kinematics Integration Test ===\n")
    
    config = TelegripConfig()
    config.enable_robot = False
    
    robot_interface = RobotInterface(config)
    robot_interface.connect()
    
    mujoco_model_path = Path(__file__).parent.parent / "mink/examples/arm620/scene.xml"
    if not mujoco_model_path.exists():
        print(f"❌ MuJoCo model not found: {mujoco_model_path}")
        return

    print(f"设置Mink运动学...")
    print(f"  - MuJoCo模型: {mujoco_model_path}")
    print()
    
    success = robot_interface.setup_mink_kinematics(
        mujoco_scene_path=str(mujoco_model_path),
        end_effector_site="tools_link"
    )
    
    if not success:
        print("❌ Mink运动学设置失败")
        return
    
    print("✓ Mink运动学设置成功\n")
    
    print("测试IK求解...")
    target_positions = [
        np.array([0.3, 0.0, 0.4]),
        np.array([0.35, 0.1, 0.45]),
        np.array([0.32, -0.05, 0.42]),
    ]
    
    for i, target_pos in enumerate(target_positions):
        print(f"\n目标 {i+1}: {target_pos}")
        
        for arm in ['left', 'right']:
            try:
                q_solution = robot_interface.solve_ik(arm, target_pos)
                print(f"  {arm.capitalize()} arm IK solution (deg): {q_solution[:3]}")
            except Exception as e:
                print(f"  {arm.capitalize()} arm IK failed: {e}")
    
    print("\n✓ Mink集成测试完成!")
    print("\n使用方法:")
    print("  1. 调用 robot_interface.setup_mink_kinematics() 设置Mink求解器")
    print("  2. 调用 robot_interface.solve_ik() 使用Mink IK")


if __name__ == "__main__":
    main()
