"""
Compatibility module for kinematics functions.
Provides helper functions kept for legacy compatibility.
"""

import numpy as np


def vr_to_robot_coordinates(vr_pos: dict, scale: float = 1.0) -> np.ndarray:
    """
    Convert VR controller position to robot coordinate system.
    
    VR coordinate system: X=right, Y=up, Z=back (towards user)
    Robot coordinate system: X=forward, Y=left, Z=up
    """
    return np.array([
        vr_pos['x'] * scale,
        -vr_pos['z'] * scale,
        vr_pos['y'] * scale
    ])


def compute_relative_position(current_vr_pos: dict, origin_vr_pos: dict, scale: float = 1.0) -> np.ndarray:
    """Compute relative position from VR origin to current position."""
    delta_vr = {
        'x': current_vr_pos['x'] - origin_vr_pos['x'],
        'y': current_vr_pos['y'] - origin_vr_pos['y'], 
        'z': current_vr_pos['z'] - origin_vr_pos['z']
    }
    return vr_to_robot_coordinates(delta_vr, scale)
