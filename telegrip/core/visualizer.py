"""
MuJoCo visualization module using mink library.
"""

import numpy as np
import mujoco
import mujoco.viewer
from typing import Dict, Optional, Tuple
import logging
from pathlib import Path
import threading

logger = logging.getLogger(__name__)


class MuJoCoVisualizer:
    """MuJoCo-based visualization for dual-arm robot teleoperation."""
    
    def __init__(self, scene_xml_path: str, use_gui: bool = True, log_level: str = "warning"):
        """
        Initialize MuJoCo visualizer.
        
        Args:
            scene_xml_path: Path to MuJoCo scene XML file (should contain both robot arms)
            use_gui: Whether to show GUI (if False, runs headless)
            log_level: Logging level
        """
        self.scene_xml_path = Path(scene_xml_path)
        self.use_gui = use_gui
        self.log_level = log_level
        
        if not self.scene_xml_path.exists():
            raise FileNotFoundError(f"MuJoCo scene file not found: {scene_xml_path}")
        
        # MuJoCo state
        self.model = None
        self.data = None
        self.viewer = None
        self.viewer_thread = None
        self.is_connected = False
        self.is_running = False
        self._mujoco_lock = threading.Lock()
        
        # Robot identifiers
        self.left_body_id = None
        self.right_body_id = None
        
        # End effector site IDs
        self.left_ee_site_id = None
        self.right_ee_site_id = None
        
        # Target marker IDs
        self.left_target_geom_id = None
        self.right_target_geom_id = None
        self.mocap_body_ids: Dict[str, int] = {}
        self._missing_marker_warnings = set()
        
        # Joint limits
        self.joint_limits_min_deg = np.full(6, -180.0)
        self.joint_limits_max_deg = np.full(6, 180.0)
        
        # Number of joints per arm
        self.num_joints = 6
        self.joint_qpos_indices = {"left": None, "right": None}
        self.joint_dof_indices = {"left": None, "right": None}
        self.actuator_indices = {"left": None, "right": None}
        self.gripper_actuator_indices = {"left": None, "right": None}
        # Default to open gripper on startup (Robotiq actuator command 0.0=open).
        self.desired_gripper_ctrl = {"left": 0.0, "right": 0.0}
        self.desired_joint_angles_deg = {
            "left": np.zeros(self.num_joints),
            "right": np.zeros(self.num_joints),
        }

        # Match arm_arm620_dual.py PID profile more closely.
        self.kp = np.array([180.0, 200.0, 200.0, 65.0, 85.0, 25.0])
        self.kd = np.array([10.0, 30.0, 10.0, 1.5, 1.0, 1.0])
        self.torque_limits = np.array([49.0, 49.0, 49.0, 9.0, 9.0, 9.0])
        self.use_gravity_compensation = True
    
    def setup(self) -> bool:
        """Initialize MuJoCo and optionally launch viewer."""
        try:
            # Load model
            self.model = mujoco.MjModel.from_xml_path(str(self.scene_xml_path))
            self.data = mujoco.MjData(self.model)
            
            # Find robot bodies and sites
            self._find_robot_components()
            
            # Read joint limits
            self._read_joint_limits()
            
            # Launch viewer if GUI enabled
            if self.use_gui:
                self._launch_viewer()
            
            self.is_connected = True
            
            if getattr(logging, self.log_level.upper()) <= logging.INFO:
                logger.info(f"MuJoCo visualization setup complete: {self.scene_xml_path.name}")
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to setup MuJoCo visualization: {e}")
            return False
    
    def _find_robot_components(self):
        """Find robot bodies, joints, and sites in the model."""
        # Prefer exact dual names first, then fallback to dynamic resolution.
        try:
            left_sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "left/tools_link")
            right_sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "right/tools_link")
            if left_sid >= 0 and right_sid >= 0:
                self.left_ee_site_id = left_sid
                self.right_ee_site_id = right_sid
                logger.info("Resolved dual end effector sites: left/tools_link, right/tools_link")
        except Exception:
            pass

        try:
            mujoco.mj_forward(self.model, self.data)
            site_ids = []
            for sid in range(self.model.nsite):
                name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_SITE, sid)
                if name and (name == "tools_link" or name.endswith("/tools_link")):
                    site_ids.append(sid)

            if len(site_ids) >= 2:
                site_ids = sorted(site_ids, key=lambda i: self.data.site_xpos[i][1], reverse=True)
                self.left_ee_site_id = site_ids[0]
                self.right_ee_site_id = site_ids[-1]
                logger.info("Resolved dual end effector sites from *tools_link")
            elif len(site_ids) == 1:
                self.left_ee_site_id = site_ids[0]
                self.right_ee_site_id = site_ids[0]
                logger.info("Resolved single end effector site: tools_link")
            else:
                logger.warning("Could not find end effector site(s) matching tools_link")
        except Exception:
            logger.warning("Could not resolve end effector sites dynamically")
        
        # Try to find target markers if they exist
        try:
            self.left_target_geom_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, "left_target"
            )
        except:
            logger.debug("No left_target geom found")
        
        try:
            self.right_target_geom_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, "right_target"
            )
        except:
            logger.debug("No right_target geom found")

        # Cache known mocap bodies used by telegrip.
        for name in ("left_target", "right_target", "left_goal", "right_goal", "target", "goal"):
            try:
                body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
                if body_id >= 0:
                    self.mocap_body_ids[name] = body_id
            except Exception:
                pass

        # Resolve per-arm joint qpos indices (works for namespaced dual scene)
        for arm in ("left", "right"):
            namespaced_joint_names = [f"{arm}/joint{i}" for i in range(1, self.num_joints + 1)]
            namespaced_actuator_names = [f"{arm}/joint{i}_motor" for i in range(1, self.num_joints + 1)]
            try:
                indices = [self.model.joint(name).qposadr[0] for name in namespaced_joint_names]
                dof_indices = [self.model.joint(name).dofadr[0] for name in namespaced_joint_names]
                act_indices = [self.model.actuator(name).id for name in namespaced_actuator_names]
                self.joint_qpos_indices[arm] = indices
                self.joint_dof_indices[arm] = dof_indices
                self.actuator_indices[arm] = act_indices
                logger.info(f"Resolved {arm} joint indices by name: {indices}")

                # Initialize desired state from current model state.
                self.desired_joint_angles_deg[arm] = np.rad2deg(self.data.qpos[indices]).copy()
            except Exception:
                self.joint_qpos_indices[arm] = None
                self.joint_dof_indices[arm] = None
                self.actuator_indices[arm] = None

            # Optional Robotiq gripper actuator (separate from joint6 motors).
            self.gripper_actuator_indices[arm] = None
            for act_name in (f"{arm}/robotiq_2f85_v4_actuator", "robotiq_2f85_v4_actuator"):
                try:
                    aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
                    if aid >= 0:
                        self.gripper_actuator_indices[arm] = int(aid)
                        logger.info(f"Resolved {arm} gripper actuator: {act_name}")
                        break
                except Exception:
                    continue
    
    def _read_joint_limits(self):
        """Read joint limits from MuJoCo model."""
        if getattr(logging, self.log_level.upper()) <= logging.INFO:
            logger.info("Reading MuJoCo joint limits:")
        
        for i in range(min(self.num_joints, self.model.njnt)):
            jnt_type = self.model.jnt_type[i]
            if jnt_type in [mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE]:
                jnt_limited = self.model.jnt_limited[i]
                if jnt_limited:
                    lower, upper = self.model.jnt_range[i]
                    self.joint_limits_min_deg[i] = np.rad2deg(lower)
                    self.joint_limits_max_deg[i] = np.rad2deg(upper)
                    
                    if getattr(logging, self.log_level.upper()) <= logging.INFO:
                        jnt_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
                        logger.info(f"  Joint {i} ({jnt_name}): {self.joint_limits_min_deg[i]:.1f}° to {self.joint_limits_max_deg[i]:.1f}°")
    
    def _launch_viewer(self):
        """Launch MuJoCo viewer in a separate thread."""
        def viewer_thread_func():
            try:
                with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                    self.viewer = viewer
                    self.is_running = True

                    # Directly render site coordinate frames and highlight both tools_link sites.
                    try:
                        viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE

                        # Hide all site groups, then show groups used by resolved EE sites.
                        for i in range(len(viewer.opt.sitegroup)):
                            viewer.opt.sitegroup[i] = 0

                        for sid in (self.left_ee_site_id, self.right_ee_site_id):
                            if sid is None or sid < 0:
                                continue
                            self.model.site_size[sid][0] = max(float(self.model.site_size[sid][0]), 0.02)
                            self.model.site_rgba[sid] = np.array([1.0, 1.0, 0.1, 1.0])
                            grp = int(self.model.site_group[sid])
                            if 0 <= grp < len(viewer.opt.sitegroup):
                                viewer.opt.sitegroup[grp] = 1
                    except Exception as e:
                        logger.debug(f"Could not configure site-frame visualization: {e}")
                    
                    # Main viewer loop
                    while self.is_running and viewer.is_running():
                        # Sync viewer with data
                        with self._mujoco_lock:
                            viewer.sync()
                        
                        # Keep viewer refresh high to match arm_arm620_dual responsiveness.
                        import time
                        time.sleep(0.001)
                    
                    self.is_running = False
                    
            except Exception as e:
                logger.error(f"Viewer thread error: {e}")
                self.is_running = False
        
        # Start viewer in background thread
        self.viewer_thread = threading.Thread(target=viewer_thread_func, daemon=True)
        self.viewer_thread.start()
        
        # Wait a bit for viewer to initialize
        import time
        time.sleep(0.5)
        
        if getattr(logging, self.log_level.upper()) <= logging.INFO:
            logger.info("MuJoCo viewer launched in background")
    
    def update_robot_pose(self, joint_angles_deg: np.ndarray, arm: str = 'left'):
        """
        Update robot joint positions in visualization.
        
        Args:
            joint_angles_deg: Joint angles in degrees
            arm: Which arm to update ('left' or 'right')
        """
        if not self.is_connected:
            return
        
        with self._mujoco_lock:
            incoming = np.asarray(joint_angles_deg[:self.num_joints], dtype=float)
            self.desired_joint_angles_deg[arm] = incoming.copy()

    def set_initial_arm_pose_deg(self, left_angles_deg: np.ndarray, right_angles_deg: np.ndarray):
        """Set initial arm posture directly in MuJoCo state and desired targets."""
        if not self.is_connected:
            return

        with self._mujoco_lock:
            l_qidx = self.joint_qpos_indices.get("left")
            r_qidx = self.joint_qpos_indices.get("right")
            if l_qidx is None or r_qidx is None:
                return

            l = np.asarray(left_angles_deg[:self.num_joints], dtype=float)
            r = np.asarray(right_angles_deg[:self.num_joints], dtype=float)

            self.data.qvel[:] = 0.0
            self.data.qpos[l_qidx] = np.deg2rad(l)
            self.data.qpos[r_qidx] = np.deg2rad(r)
            mujoco.mj_forward(self.model, self.data)

            self.desired_joint_angles_deg["left"] = l.copy()
            self.desired_joint_angles_deg["right"] = r.copy()
    
    def _resolve_mocap_id(self, marker_name: str) -> Optional[int]:
        """Resolve marker name to a mocap body id with single-arm fallbacks."""
        body_id = self.mocap_body_ids.get(marker_name)

        # Fallback mapping for single-arm scenes.
        if body_id is None:
            if marker_name in ("left_target", "right_target"):
                body_id = self.mocap_body_ids.get("target")
            elif marker_name in ("left_goal", "right_goal"):
                body_id = self.mocap_body_ids.get("goal")

        return body_id

    def update_marker_position(self, marker_name: str, position: np.ndarray,
                              orientation: Optional[np.ndarray] = None) -> bool:
        """
        Update position of a visualization marker.
        
        Args:
            marker_name: Name of marker (e.g., 'left_target', 'right_target')
            position: Position [x, y, z]
            orientation: Quaternion [w, x, y, z] (optional)
        """
        if not self.is_connected:
            return False

        # left/right_actual are optional debug markers and may not exist in scene XML.
        # Keep this path silent to avoid log spam.
        if marker_name in ("left_actual", "right_actual"):
            return False
        
        try:
            with self._mujoco_lock:
                body_id = self._resolve_mocap_id(marker_name)
                if body_id is None:
                    if marker_name not in self._missing_marker_warnings:
                        self._missing_marker_warnings.add(marker_name)
                        logger.warning(f"Marker body not found in scene: {marker_name}")
                    return False

                mocap_id = self.model.body(body_id).mocapid[0]
                if mocap_id >= 0:
                    self.data.mocap_pos[mocap_id] = position
                    if orientation is not None:
                        self.data.mocap_quat[mocap_id] = orientation
                    return True
        except:
            pass

        return False

    def get_mocap_position(self, marker_name: str) -> Optional[np.ndarray]:
        """Read current mocap marker position from MuJoCo state."""
        if not self.is_connected:
            return None

        with self._mujoco_lock:
            body_id = self._resolve_mocap_id(marker_name)
            if body_id is None:
                return None

            mocap_id = self.model.body(body_id).mocapid[0]
            if mocap_id < 0:
                return None
            return self.data.mocap_pos[mocap_id].copy()

    def get_mocap_quaternion(self, marker_name: str) -> Optional[np.ndarray]:
        """Read current mocap marker quaternion [w, x, y, z] from MuJoCo state."""
        if not self.is_connected:
            return None

        with self._mujoco_lock:
            body_id = self._resolve_mocap_id(marker_name)
            if body_id is None:
                return None

            mocap_id = self.model.body(body_id).mocapid[0]
            if mocap_id < 0:
                return None
            return self.data.mocap_quat[mocap_id].copy()
    
    def update_coordinate_frame(self, frame_name: str, position: np.ndarray, 
                               orientation_quat: Optional[np.ndarray] = None):
        """
        Update coordinate frame visualization.
        
        Note: MuJoCo doesn't have a direct coordinate-frame primitive.
        This is handled automatically through the scene graph.
        """
        # In MuJoCo, coordinate frames are automatically visualized if sites are defined
        pass
    
    def hide_marker(self, marker_name: str):
        """Hide a marker by moving it off-screen."""
        self.update_marker_position(marker_name, np.array([0, 0, -10]))
    
    def hide_frame(self, frame_name: str):
        """Hide a coordinate frame."""
        # Not needed for MuJoCo - frames visibility controlled by XML
        pass

    def set_gripper_closed(self, arm: str, closed: bool):
        """Set desired gripper state for arm with Robotiq-style actuator."""
        with self._mujoco_lock:
            self.desired_gripper_ctrl[arm] = 255.0 if bool(closed) else 0.0
    
    def step_simulation(self, substeps: int = 1):
        """Step the simulation forward.

        Args:
            substeps: Number of MuJoCo integration substeps to run.
        """
        if self.is_connected:
            with self._mujoco_lock:
                n = max(1, int(substeps))
                for _ in range(n):
                    # Apply PD torque control for both arms (recomputed every sim step).
                    for arm in ("left", "right"):
                        qidx = self.joint_qpos_indices.get(arm)
                        didx = self.joint_dof_indices.get(arm)
                        aidx = self.actuator_indices.get(arm)
                        if qidx is None or didx is None or aidx is None:
                            continue

                        q = self.data.qpos[qidx]
                        dq = self.data.qvel[didx]
                        q_target = np.deg2rad(self.desired_joint_angles_deg[arm])

                        tau = self.kp * (q_target - q) - self.kd * dq

                        # Gravity compensation from MuJoCo dynamics bias.
                        if self.use_gravity_compensation:
                            tau = tau + self.data.qfrc_bias[didx]

                        tau = np.clip(tau, -self.torque_limits, self.torque_limits)
                        self.data.ctrl[aidx] = tau

                        # Optional separate gripper actuator command (e.g., Robotiq 2F-85).
                        gidx = self.gripper_actuator_indices.get(arm)
                        if gidx is not None and gidx >= 0 and gidx < self.model.nu:
                            self.data.ctrl[gidx] = float(self.desired_gripper_ctrl.get(arm, 0.0))

                    mujoco.mj_step(self.model, self.data)
    
    def disconnect(self):
        """Disconnect and cleanup."""
        self.is_running = False
        
        if self.viewer_thread and self.viewer_thread.is_alive():
            self.viewer_thread.join(timeout=2.0)
        
        self.is_connected = False
        
        if getattr(logging, self.log_level.upper()) <= logging.INFO:
            logger.info("MuJoCo visualizer disconnected")
    
    @property
    def get_joint_limits(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get joint limits in degrees."""
        return self.joint_limits_min_deg.copy(), self.joint_limits_max_deg.copy()
    
    def get_physics_client(self):
        """Return model for compatibility with existing code."""
        return self.model
    
    def get_robot_id(self, arm: str):
        """Return data for compatibility - MuJoCo uses single data instance."""
        return self.data
    
    def get_joint_indices(self, arm: str):
        """Return joint indices for specified arm."""
        if arm == 'left':
            return list(range(self.num_joints))
        elif arm == 'right':
            return list(range(self.num_joints, 2 * self.num_joints))
        return []
    
    def get_end_effector_link_index(self, arm: str):
        """Return end effector site ID."""
        if arm == 'left':
            return self.left_ee_site_id
        elif arm == 'right':
            return self.right_ee_site_id
        return -1

    def get_end_effector_position(self, arm: str) -> Optional[np.ndarray]:
        """Get current end-effector position from simulation state."""
        if not self.is_connected:
            return None

        site_id = self.left_ee_site_id if arm == "left" else self.right_ee_site_id
        if site_id is None or site_id < 0:
            return None

        with self._mujoco_lock:
            return self.data.site_xpos[site_id].copy()

    def get_end_effector_quaternion(self, arm: str) -> Optional[np.ndarray]:
        """Get current end-effector orientation quaternion [w, x, y, z]."""
        if not self.is_connected:
            return None

        site_id = self.left_ee_site_id if arm == "left" else self.right_ee_site_id
        if site_id is None or site_id < 0:
            return None

        with self._mujoco_lock:
            quat = np.zeros(4, dtype=float)
            mujoco.mju_mat2Quat(quat, self.data.site_xmat[site_id])
            return quat.copy()

    def get_joint_angles_deg(self, arm: str) -> Optional[np.ndarray]:
        """Get current simulated joint angles (degrees) for one arm."""
        if not self.is_connected:
            return None

        qidx = self.joint_qpos_indices.get(arm)
        if qidx is None:
            return None

        with self._mujoco_lock:
            return np.rad2deg(self.data.qpos[qidx]).copy()
