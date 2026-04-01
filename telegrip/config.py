"""
Configuration module for the unified teleoperation system.
Loads configuration from config.yaml file with fallback to default values.
"""

import os
import yaml
from dataclasses import dataclass
from typing import List, Optional
import numpy as np
from pathlib import Path
import logging
from .utils import get_absolute_path, get_project_root

logger = logging.getLogger(__name__)

# Default configuration values (fallback if YAML file doesn't exist)
DEFAULT_CONFIG = {
    "network": {
        "https_port": 8443,
        "websocket_port": 8442,
        "host_ip": "0.0.0.0"
    },
    "gnirehtet": {
        "enabled": False,
        "binary": "gnirehtet-rust-linux64/gnirehtet",
        "mode": "run",
        "args": []
    },
    "ssl": {
        "certfile": "cert.pem",
        "keyfile": "key.pem"
    },
    "robot": {
        "left_arm": {
            "name": "Left Arm",
            "enabled": True
        },
        "right_arm": {
            "name": "Right Arm",
            "enabled": True
        },
        "command_backend": "ros2_topic",
        "arm_controller": {
            "workspace": "/home/nvidia/Projects/universal-arm-controller",
            "module_dir": "",
            "left_mapping": "left_arm",
            "right_mapping": "right_arm",
            "interpolation_alpha": 1.0,
            "max_step_deg": 0.0,
        },
        "vr_to_robot_scale": 1.0,
        "send_interval": 0.05,
        "require_state_feedback": False,
        # Hard gate for real-robot motion: block commands until /joint_states is received.
        "require_joint_state_for_motion": True,
    },
    "control": {
        "use_mink": True,
        "enable_sim": True,
        "enable_gui": True,
        "mink": {
            "enabled": True,
            "mujoco_scene": "mink/examples/arm620/scene.xml",
            "end_effector_site": "tools_link",
            "position_cost": 1.0,
            "orientation_cost": 1.0,
            "posture_cost": 0.01,
            "lm_damping": 1.0,
            "solve_damping": 0.001,
            "dt": 0.005,
            "max_iters": 20,
            "position_error_threshold": 1e-4,
            "orientation_error_threshold": 1e-3,
        },
        "keyboard": {
            "enabled": False,
            "pos_step": 0.01,
            "angle_step": 5.0,
            "gripper_step": 10.0
        },
        "vr": {
            "enabled": True,
            "orientation_reference_mode": "global_calibration",
            "orientation_reference_quat_xyzw": {},
            # Optional 3x3 axis remap matrix for relative-rotation delta in control loop.
            # Keep [] to disable and use identity.
            "relative_rotation_axis_map": []
        }
    },
    "paths": {
        "urdf_path": "URDF/SO100/so100.urdf"
    },
    "gripper": {
        "open_angle": 0.0,
        "closed_angle": 45.0
    },
    "ik": {
        "use_reference_poses": True,
        "reference_poses_file": "reference_poses.json",
        "position_error_threshold": 0.001,
        "hysteresis_threshold": 0.01,
        "movement_penalty_weight": 0.01
    }
}

def load_config(config_path: str = "config.yaml") -> dict:
    """Load configuration from YAML file with fallback to defaults."""
    config = DEFAULT_CONFIG.copy()
    
    # Try to load from project root first (package installation directory)
    package_config_path = get_absolute_path(config_path)
    
    # Check if config exists in package directory
    if package_config_path.exists():
        config_file_to_use = package_config_path
        logger.info(f"Loading config from package directory: {config_file_to_use}")
    # Fallback to current working directory (for user-provided configs)
    elif os.path.exists(config_path):
        config_file_to_use = Path(config_path)
        logger.info(f"Loading config from current directory: {config_file_to_use}")
    else:
        logger.info(f"Config file {config_path} not found in package directory ({package_config_path}) or current directory, using defaults")
        return config
    
    try:
        with open(config_file_to_use, 'r') as f:
            yaml_config = yaml.safe_load(f)
            if yaml_config:
                # Deep merge yaml config into default config
                _deep_merge(config, yaml_config)
    except Exception as e:
        logger.warning(f"Could not load config from {config_file_to_use}: {e}")
        logger.info("Using default configuration")
    
    return config

def save_config(config: dict, config_path: str = "config.yaml"):
    """Save configuration to YAML file in project root."""
    # Always save to project root directory
    abs_config_path = get_absolute_path(config_path)
    try:
        with open(abs_config_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False, indent=2)
        return True
    except Exception as e:
        logger.error(f"Error saving config to {abs_config_path}: {e}")
        return False

def _deep_merge(base: dict, update: dict):
    """Deep merge update dict into base dict."""
    for key, value in update.items():
        if key in base and isinstance(base[key], dict) and isinstance(value, dict):
            _deep_merge(base[key], value)
        else:
            base[key] = value

# Load configuration
_config_data = load_config()

# Extract values for backward compatibility
HTTPS_PORT = _config_data["network"]["https_port"]
WEBSOCKET_PORT = _config_data["network"]["websocket_port"]
HOST_IP = _config_data["network"]["host_ip"]

CERTFILE = _config_data["ssl"]["certfile"]
KEYFILE = _config_data["ssl"]["keyfile"]

VR_TO_ROBOT_SCALE = _config_data["robot"]["vr_to_robot_scale"]
SEND_INTERVAL = _config_data["robot"]["send_interval"]

POS_STEP = _config_data["control"]["keyboard"]["pos_step"]
ANGLE_STEP = _config_data["control"]["keyboard"]["angle_step"]
GRIPPER_STEP = _config_data["control"]["keyboard"]["gripper_step"]

USE_MINK = _config_data["control"].get("use_mink", True)
MINK_MUJOCO_SCENE = _config_data["control"].get("mink", {}).get("mujoco_scene", "mink/examples/arm620/scene.xml")
MINK_END_EFFECTOR_SITE = _config_data["control"].get("mink", {}).get("end_effector_site", "tools_link")
MINK_POSITION_COST = float(_config_data["control"].get("mink", {}).get("position_cost", 1.0))
MINK_ORIENTATION_COST = float(_config_data["control"].get("mink", {}).get("orientation_cost", 1.0))
MINK_POSTURE_COST = float(_config_data["control"].get("mink", {}).get("posture_cost", 0.01))
MINK_LM_DAMPING = float(_config_data["control"].get("mink", {}).get("lm_damping", 1.0))
MINK_SOLVE_DAMPING = float(_config_data["control"].get("mink", {}).get("solve_damping", 1e-3))
MINK_DT = float(_config_data["control"].get("mink", {}).get("dt", 0.005))
MINK_MAX_ITERS = int(_config_data["control"].get("mink", {}).get("max_iters", 20))
MINK_POS_THRESHOLD = float(_config_data["control"].get("mink", {}).get("position_error_threshold", 1e-4))
MINK_ORI_THRESHOLD = float(_config_data["control"].get("mink", {}).get("orientation_error_threshold", 1e-3))

URDF_PATH = _config_data["paths"]["urdf_path"]

GRIPPER_OPEN_ANGLE = _config_data["gripper"]["open_angle"]
GRIPPER_CLOSED_ANGLE = _config_data["gripper"]["closed_angle"]

# IK Configuration
USE_REFERENCE_POSES = _config_data["ik"]["use_reference_poses"]
REFERENCE_POSES_FILE = _config_data["ik"]["reference_poses_file"]
IK_POSITION_ERROR_THRESHOLD = _config_data["ik"]["position_error_threshold"]
IK_HYSTERESIS_THRESHOLD = _config_data["ik"]["hysteresis_threshold"]
IK_MOVEMENT_PENALTY_WEIGHT = _config_data["ik"]["movement_penalty_weight"]

# --- Joint Configuration ---
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
NUM_JOINTS = len(JOINT_NAMES)
NUM_IK_JOINTS = 3  # Use only first 3 joints for IK (Rotation, Pitch, Elbow)
WRIST_FLEX_INDEX = 3
WRIST_ROLL_INDEX = 4
GRIPPER_INDEX = 5

# Motor configuration for SO100
COMMON_MOTORS = {
    "shoulder_pan": [1, "sts3215"],
    "shoulder_lift": [2, "sts3215"], 
    "elbow_flex": [3, "sts3215"],
    "wrist_flex": [4, "sts3215"],
    "wrist_roll": [5, "sts3215"],
    "gripper": [6, "sts3215"],
}

# URDF joint name mapping
URDF_TO_INTERNAL_NAME_MAP = {
    "1": "shoulder_pan",
    "2": "shoulder_lift",
    "3": "elbow_flex",
    "4": "wrist_flex",
    "5": "wrist_roll",
    "6": "gripper",
}

# --- End Effector Configuration ---
END_EFFECTOR_LINK_NAME = "Fixed_Jaw_tip"

# --- Input Step Defaults ---
POS_STEP = 0.01  # meters
ANGLE_STEP = 5.0 # degrees
GRIPPER_STEP = 10.0 # degrees

@dataclass
class TelegripConfig:
    """Main configuration class for the teleoperation system."""
    
    # Network settings
    https_port: int = HTTPS_PORT
    websocket_port: int = WEBSOCKET_PORT
    host_ip: str = HOST_IP

    # Gnirehtet settings
    gnirehtet_enabled: bool = bool(_config_data.get("gnirehtet", {}).get("enabled", False))
    gnirehtet_binary: str = str(_config_data.get("gnirehtet", {}).get("binary", "gnirehtet-rust-linux64/gnirehtet"))
    gnirehtet_mode: str = str(_config_data.get("gnirehtet", {}).get("mode", "run"))
    gnirehtet_args: List[str] = None
    
    # SSL settings
    certfile: str = CERTFILE
    keyfile: str = KEYFILE
    
    # Robot settings
    vr_to_robot_scale: float = VR_TO_ROBOT_SCALE
    send_interval: float = SEND_INTERVAL
    require_state_feedback: bool = _config_data["robot"].get("require_state_feedback", False)
    require_joint_state_for_motion: bool = bool(
        _config_data["robot"].get("require_joint_state_for_motion", True)
    )
    robot_command_backend: str = str(_config_data["robot"].get("command_backend", "ros2_topic"))
    arm_controller_workspace: str = str(
        _config_data["robot"].get("arm_controller", {}).get("workspace", "/home/nvidia/Projects/universal-arm-controller")
    )
    arm_controller_module_dir: str = str(
        _config_data["robot"].get("arm_controller", {}).get("module_dir", "")
    )
    arm_controller_left_mapping: str = str(
        _config_data["robot"].get("arm_controller", {}).get("left_mapping", "left_arm")
    )
    arm_controller_right_mapping: str = str(
        _config_data["robot"].get("arm_controller", {}).get("right_mapping", "right_arm")
    )
    arm_command_interpolation_alpha: float = float(
        _config_data["robot"].get("arm_controller", {}).get("interpolation_alpha", 1.0)
    )
    arm_command_max_step_deg: float = float(
        _config_data["robot"].get("arm_controller", {}).get("max_step_deg", 0.0)
    )
    
    # Control flags
    enable_sim: bool = True
    enable_gui: bool = True
    enable_robot: bool = True
    enable_vr: bool = True
    vr_orientation_reference_mode: str = str(
        _config_data.get("control", {}).get("vr", {}).get("orientation_reference_mode", "global_calibration")
    )
    vr_relative_rotation_axis_map: Optional[List[List[float]]] = None
    enable_keyboard: bool = False
    autoconnect: bool = False
    log_level: str = "warning"

    # Mink control settings
    use_mink: bool = USE_MINK
    mink_mujoco_scene: str = MINK_MUJOCO_SCENE
    end_effector_site: str = MINK_END_EFFECTOR_SITE
    mink_position_cost: float = MINK_POSITION_COST
    mink_orientation_cost: float = MINK_ORIENTATION_COST
    mink_posture_cost: float = MINK_POSTURE_COST
    mink_lm_damping: float = MINK_LM_DAMPING
    mink_solve_damping: float = MINK_SOLVE_DAMPING
    mink_dt: float = MINK_DT
    mink_max_iters: int = MINK_MAX_ITERS
    mink_position_error_threshold: float = MINK_POS_THRESHOLD
    mink_orientation_error_threshold: float = MINK_ORI_THRESHOLD
    
    # Paths
    urdf_path: str = URDF_PATH
    webapp_dir: str = "webapp"
    
    # IK settings
    use_reference_poses: bool = USE_REFERENCE_POSES
    reference_poses_file: str = REFERENCE_POSES_FILE
    ik_position_error_threshold: float = IK_POSITION_ERROR_THRESHOLD
    ik_hysteresis_threshold: float = IK_HYSTERESIS_THRESHOLD
    ik_movement_penalty_weight: float = IK_MOVEMENT_PENALTY_WEIGHT
    
    # Gripper settings
    gripper_open_angle: float = GRIPPER_OPEN_ANGLE
    gripper_closed_angle: float = GRIPPER_CLOSED_ANGLE
    
    # Input step defaults
    pos_step: float = POS_STEP
    angle_step: float = ANGLE_STEP
    gripper_step: float = GRIPPER_STEP
    
    def __post_init__(self):
        if self.gnirehtet_args is None:
            cfg_args = _config_data.get("gnirehtet", {}).get("args", [])
            self.gnirehtet_args = [str(arg) for arg in cfg_args] if isinstance(cfg_args, list) else []
        if self.vr_relative_rotation_axis_map is None:
            cfg_map = _config_data.get("control", {}).get("vr", {}).get("relative_rotation_axis_map", [])
            if isinstance(cfg_map, list):
                self.vr_relative_rotation_axis_map = cfg_map.copy()
            else:
                self.vr_relative_rotation_axis_map = []
        self.robot_command_backend = str(self.robot_command_backend or "ros2_topic").strip().lower()
        self.require_joint_state_for_motion = bool(self.require_joint_state_for_motion)
        self.arm_controller_left_mapping = str(self.arm_controller_left_mapping or "left_arm").strip()
        self.arm_controller_right_mapping = str(self.arm_controller_right_mapping or "right_arm").strip()
        self.arm_command_interpolation_alpha = float(np.clip(self.arm_command_interpolation_alpha, 0.0, 1.0))
        self.arm_command_max_step_deg = max(0.0, float(self.arm_command_max_step_deg))
    
    @property
    def ssl_files_exist(self) -> bool:
        """Check if SSL certificate files exist."""
        cert_path = get_absolute_path(self.certfile)
        key_path = get_absolute_path(self.keyfile)
        return cert_path.exists() and key_path.exists()
    
    def ensure_ssl_certificates(self) -> bool:
        """Ensure SSL certificates exist, generating them if necessary."""
        from .utils import ensure_ssl_certificates
        return ensure_ssl_certificates(self.certfile, self.keyfile)
    
    @property
    def urdf_exists(self) -> bool:
        """Check if URDF file exists."""
        urdf_path = get_absolute_path(self.urdf_path)
        return urdf_path.exists()
    
    @property
    def webapp_exists(self) -> bool:
        """Check if webapp directory exists."""
        webapp_path = get_absolute_path(self.webapp_dir)
        return webapp_path.exists()
    
    def get_absolute_urdf_path(self) -> str:
        """Get absolute path to URDF file."""
        return str(get_absolute_path(self.urdf_path))
    
    def get_absolute_reference_poses_path(self) -> str:
        """Get absolute path to reference poses file."""
        return str(get_absolute_path(self.reference_poses_file))
    
    def get_absolute_ssl_paths(self) -> tuple:
        """Get absolute paths to SSL certificate files."""
        cert_path = str(get_absolute_path(self.certfile))
        key_path = str(get_absolute_path(self.keyfile))
        return cert_path, key_path

def get_config_data():
    """Get the current configuration data."""
    return _config_data.copy()

def update_config_data(new_config: dict):
    """Update the global configuration data."""
    global _config_data
    _config_data = new_config
    
    # Save to file
    save_config(_config_data)

# Global configuration instance
config = TelegripConfig() 
