#!/usr/bin/env python3
"""
VR Teleoperation for ARM620 Robot

This example demonstrates VR-based teleoperation of dual ARM620 robotic arms
using the telegrip framework with Mink inverse kinematics.

Usage:
    python vr_teleoperate_arm620.py [options]

    --no-robot          Disable robot connection (visualization only)
    --no-sim            Disable PyBullet simulation
    --no-viz            Disable PyBullet visualization (headless mode)
    --no-vr             Disable VR WebSocket server
    --no-keyboard       Disable keyboard input
    --autoconnect       Automatically connect to robot motors on startup
    --log-level LEVEL   Set logging level (debug, info, warning, error, critical)
    --config FILE       Path to config file (default: config.yaml)

Requirements:
    - ARM620 URDF model in mink/examples/arm620/urdf/
    - MuJoCo scene file in mink/examples/arm620/scene.xml
    - VR headset with WebXR support
    - HTTPS server for WebXR (runs automatically)

Features:
    - Dual ARM620 control via VR controllers
    - Real-time inverse kinematics using Mink
    - Gripper control via VR triggers
    - Keyboard fallback control
    - Web-based UI for monitoring and control
"""

import numpy as np
import sys
import logging
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from telegrip.main import main_cli, parse_arguments, create_config_from_args
from telegrip.config import TelegripConfig

logger = logging.getLogger(__name__)


def setup_arm620_config():
    """Setup configuration for ARM620 robot."""
    args = parse_arguments()
    
    config = create_config_from_args(args)
    
    mujoco_model_path = Path(__file__).parent.parent / "mink/examples/arm620/scene.xml"
    urdf_path = Path(__file__).parent.parent / "mink/examples/arm620/urdf/arm620.urdf"
    
    if not mujoco_model_path.exists():
        logger.error(f"MuJoCo model not found: {mujoco_model_path}")
        logger.error("Please ensure ARM620 model files are in mink/examples/arm620/")
        sys.exit(1)
    
    if not urdf_path.exists():
        logger.error(f"URDF not found: {urdf_path}")
        logger.error("Please ensure ARM620 URDF is in mink/examples/arm620/urdf/")
        sys.exit(1)
    
    config.mujoco_model_path = str(mujoco_model_path)
    config.urdf_path = str(urdf_path)
    config.end_effector_site = "tools_link"
    
    config.kp = np.array([180, 200, 200, 65, 85, 25])
    config.kd = np.array([10, 30, 10, 1.5, 1, 1])
    config.ki = np.array([0, 0, 0, 0, 0, 0])
    
    return config


if __name__ == "__main__":
    print("=" * 60)
    print("VR Teleoperation for ARM620 Robot")
    print("=" * 60)
    print()
    print("Setup:")
    print("  1. Put on your VR headset")
    print("  2. Open the browser on your VR headset")
    print("  3. Navigate to the HTTPS URL shown below")
    print("  4. Click 'Enter VR' button in the web interface")
    print()
    print("Controls:")
    print("  VR Controllers: Move end effectors")
    print("  VR Triggers: Open/close grippers")
    print("  Web UI: Connect/disconnect robot, enable keyboard")
    print()
    print("Starting teleoperation system...")
    print()
    
    main_cli()
