"""
Main control loop for the teleoperation system.
Consumes control goals from the command queue and executes them via the robot interface.
"""

import asyncio
import numpy as np
import logging
import time
import queue  # Add import for thread-safe queue
import sys
from typing import Dict, Optional
from pathlib import Path

from .config import TelegripConfig, NUM_JOINTS, WRIST_FLEX_INDEX, WRIST_ROLL_INDEX, GRIPPER_INDEX
from .core.robot_interface import RobotInterface
# MuJoCoVisualizer will be imported on demand
from .inputs.base import ControlGoal, ControlMode
# Optional input handlers are imported on demand to avoid circular imports

logger = logging.getLogger(__name__)

# Match mink/examples/arm_arm620_dual.py behavior.
CONTROL_HZ = 200.0
CONTROL_DT = 1.0 / CONTROL_HZ
IK_TARGET_SMOOTHING = 1.0


class ArmState:
    """State tracking for a single robot arm."""
    
    def __init__(self, arm_name: str):
        self.arm_name = arm_name
        self.mode = ControlMode.IDLE
        self.target_position = None
        self.target_orientation_quat = None
        self.origin_target_orientation_quat = None
        self.goal_position = None  # For visualization
        self.origin_position = None  # Robot position when grip was activated
        self.origin_wrist_roll_angle = 0.0
        self.origin_wrist_flex_angle = 0.0
        self.current_wrist_roll = 0.0
        self.current_wrist_flex = 0.0
        self.last_mocap_target_position = None
        self.last_mocap_target_quaternion = None
        # Fixed offset for absolute-controller orientation mapping.
        self.controller_to_target_offset_quat = None
        # Global orientation reference captured at system initialization.
        self.initial_target_orientation_quat = None
        
    def reset(self):
        """Reset arm state to idle."""
        self.mode = ControlMode.IDLE
        self.target_position = None
        self.target_orientation_quat = None
        self.origin_target_orientation_quat = None
        self.goal_position = None
        self.origin_position = None
        self.origin_wrist_roll_angle = 0.0
        self.origin_wrist_flex_angle = 0.0
        self.last_mocap_target_position = None
        self.last_mocap_target_quaternion = None
        self.controller_to_target_offset_quat = None


class ControlLoop:
    """Main control loop that processes command queue and controls robot."""
    
    def __init__(self, command_queue: asyncio.Queue, config: TelegripConfig, control_commands_queue: Optional[queue.Queue] = None):
        self.command_queue = command_queue
        self.control_commands_queue = control_commands_queue
        self.config = config
        
        # Components
        self.robot_interface = None
        self.visualizer = None
        self.web_keyboard_handler = None  # Optional external input handler reference
        
        # Arm states
        self.left_arm = ArmState("left")
        self.right_arm = ArmState("right")
        
        # Control timing
        self.last_log_time = 0
        self.log_interval = 1.0  # Log status every second
        self.sim_substeps = 1

        # 运行时频率统计（动态输出）。
        self._perf_window_start = time.perf_counter()
        self._tick_counter = 0
        self._mink_solve_counter = 0
        self._mujoco_substep_counter = 0
        
        # Debug flags
        self._queue_debug_logged = False
        self._process_debug_logged = False
        
        self.is_running = False
        # VR relative translation -> base_link translation remap.
        # +X(right) -> -Y, +/-Y(front/back) -> +/-X, Z unchanged.
        self._vr_delta_to_base_quat = self._quat_from_euler_xyz_deg(0.0, 0.0, -90.0)
        # Controller-delta -> target-delta orientation frame mapping.
        # The VR-side frame correction has been applied before transmission, so
        # this mapping stays identity in control loop.
        self._controller_delta_to_target_axis_map = np.array(
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            dtype=float,
        )
        # Optional extra axis remap after relative-rotation delta conversion.
        # User-configurable via control.vr.relative_rotation_axis_map (3x3).
        self._relative_rotation_post_axis_map = self._parse_axis_remap_matrix(
            getattr(self.config, "vr_relative_rotation_axis_map", [])
        )
        if not np.allclose(self._relative_rotation_post_axis_map, np.eye(3), atol=1e-9):
            logger.info(
                "Using vr.relative_rotation_axis_map:\n"
                f"{self._relative_rotation_post_axis_map}"
            )
        # Guard to ensure startup joint_state->MuJoCo alignment runs only once.
        self._startup_joint_state_initialized = False

    @staticmethod
    def _joint_state_to_degrees(raw_angles: np.ndarray) -> Optional[np.ndarray]:
        """Validate startup joint_state snapshot values (already in degrees)."""
        arr = np.asarray(raw_angles, dtype=float).reshape(-1)
        if arr.size < NUM_JOINTS:
            return None
        arr = arr[:NUM_JOINTS]
        if not np.all(np.isfinite(arr)):
            return None
        return arr.copy()

    @staticmethod
    def _parse_axis_remap_matrix(raw_matrix) -> np.ndarray:
        """Parse optional 3x3 axis remap matrix; fallback to identity on invalid input."""
        identity = np.eye(3, dtype=float)
        if raw_matrix is None:
            return identity
        if isinstance(raw_matrix, list) and len(raw_matrix) == 0:
            return identity
        try:
            mat = np.asarray(raw_matrix, dtype=float)
        except Exception:
            logger.warning("Invalid vr.relative_rotation_axis_map type; using identity")
            return identity
        if mat.shape != (3, 3) or not np.all(np.isfinite(mat)):
            logger.warning("vr.relative_rotation_axis_map must be a finite 3x3 matrix; using identity")
            return identity

        # Similarity transform S*R*S^T assumes orthonormal axis map.
        ortho_err = float(np.max(np.abs(mat @ mat.T - identity)))
        det = float(np.linalg.det(mat))
        if ortho_err > 1e-3 or abs(abs(det) - 1.0) > 1e-3:
            logger.warning(
                "vr.relative_rotation_axis_map is not an orthonormal axis map "
                f"(ortho_err={ortho_err:.3e}, det={det:.6f}); using identity"
            )
            return identity
        return mat

    def _collect_startup_joint_state_pose_deg(self, timeout_s: float = 3.0) -> Optional[Dict[str, np.ndarray]]:
        """Fetch one startup joint_state snapshot and convert to degree pose."""
        if self._startup_joint_state_initialized:
            return None
        self._startup_joint_state_initialized = True

        if not self.robot_interface:
            return None

        snapshot = self.robot_interface.wait_for_joint_state_snapshot(timeout_s=timeout_s)
        if not snapshot:
            logger.info("Startup joint_state snapshot unavailable; keeping MuJoCo default initial pose")
            return None

        pose_deg: Dict[str, np.ndarray] = {}
        for arm in ("left", "right"):
            raw = snapshot.get(arm)
            if raw is None:
                continue
            converted = self._joint_state_to_degrees(raw)
            if converted is not None:
                pose_deg[arm] = converted

        if not pose_deg:
            logger.warning("Startup joint_state snapshot received but could not parse joint values")
            return None

        logger.info(
            "Startup joint_state snapshot captured for arms: "
            + ", ".join(sorted(pose_deg.keys()))
        )
        return pose_deg

    def _seed_robot_state_from_pose_deg(self, left_deg: np.ndarray, right_deg: np.ndarray):
        """Seed robot interface commanded/simulated states from startup pose."""
        if not self.robot_interface:
            return

        l = np.asarray(left_deg[:NUM_JOINTS], dtype=float).copy()
        r = np.asarray(right_deg[:NUM_JOINTS], dtype=float).copy()
        self.robot_interface.left_arm_angles = l.copy()
        self.robot_interface.right_arm_angles = r.copy()
        self.robot_interface.set_simulated_arm_angles("left", l)
        self.robot_interface.set_simulated_arm_angles("right", r)

        if self.robot_interface.ros_node is not None:
            self.robot_interface.ros_node.left_arm_angles = l.copy()
            self.robot_interface.ros_node.right_arm_angles = r.copy()

    
    def setup(self) -> bool:
        """Setup robot interface and visualizer."""
        success = True
        setup_errors = []
        startup_joint_pose_deg = None
        
        # Setup robot interface
        try:
            self.robot_interface = RobotInterface(self.config)
            connect_result = self.robot_interface.connect()
            logger.info(f"Robot interface connect() returned: {connect_result}, enable_robot={self.config.enable_robot}")
            if not connect_result:
                error_msg = "Robot interface failed to connect"
                logger.error(error_msg)
                setup_errors.append(error_msg)
                if self.config.enable_robot:
                    success = False
                    logger.error(f"Setting success=False because enable_robot={self.config.enable_robot}")
                else:
                    logger.info(f"Not failing setup because enable_robot={self.config.enable_robot}")
        except Exception as e:
            error_msg = f"Robot interface setup failed with exception: {e}"
            logger.error(error_msg)
            setup_errors.append(error_msg)
            if self.config.enable_robot:
                success = False

        # Capture startup joint_state once so MuJoCo can start from real arm pose.
        try:
            startup_joint_pose_deg = self._collect_startup_joint_state_pose_deg(timeout_s=3.0)
        except Exception as e:
            logger.warning(f"Failed to capture startup joint_state snapshot: {e}")
            startup_joint_pose_deg = None
        
        # Setup Mink+MuJoCo visualization and IK
        # Always try to use Mink unless explicitly disabled
        try:
            from .core.visualizer import MuJoCoVisualizer
            from .utils import get_absolute_path

            # Import dual-scene builder from local mink/examples directory.
            examples_path = Path(__file__).resolve().parents[1] / "mink" / "examples"
            if str(examples_path) not in sys.path:
                sys.path.insert(0, str(examples_path))
            from arm620.scene_dual_builder import save_dual_arm620_xml
            from arm620.scene_dual_unified_base_builder import save_dual_unified_xml
            
            # Kinematics scene and end-effector site are configurable.
            kinematics_scene = self.config.mink_mujoco_scene
            end_effector_site = self.config.end_effector_site

            # Visualization scene follows config. For generated dual ARM620 scenes, write temp XML.
            dual_scene_tmp = 'mink/examples/arm620/_tmp_dual_saved.xml'
            dual_unified_scene_tmp = 'mink/examples/arm620/_tmp_dual_unified_saved.xml'
            
            kinematics_scene_path = get_absolute_path(kinematics_scene)
            scene_name = kinematics_scene_path.name.lower()
            use_dual_generated_scene = "dual" in scene_name
            use_unified_base_scene = False
            if use_dual_generated_scene:
                use_unified_base_scene = "unified" in scene_name
                dual_scene_path = get_absolute_path(
                    dual_unified_scene_tmp if use_unified_base_scene else dual_scene_tmp
                )
                dual_scene_path.parent.mkdir(parents=True, exist_ok=True)
                if use_unified_base_scene:
                    save_dual_unified_xml(dual_scene_path)
                else:
                    save_dual_arm620_xml(dual_scene_path)
                visualizer_scene_path = dual_scene_path
                # Keep IK and visualization in the same world/model frame.
                kinematics_scene_path = dual_scene_path
                logger.info(
                    "Dual visualization enabled: using generated "
                    f"{'unified-base ' if use_unified_base_scene else ''}"
                    "dual scene for Mink IK "
                    f"({kinematics_scene_path.name})"
                )
            else:
                visualizer_scene_path = kinematics_scene_path
            
            visualizer_startup_pose = None
            if (
                isinstance(startup_joint_pose_deg, dict)
                and "left" in startup_joint_pose_deg
                and "right" in startup_joint_pose_deg
            ):
                visualizer_startup_pose = startup_joint_pose_deg

            self.visualizer = MuJoCoVisualizer(
                str(visualizer_scene_path),
                use_gui=self.config.enable_gui,
                log_level=self.config.log_level,
                startup_arm_pose_deg=visualizer_startup_pose,
            )
            if not self.visualizer.setup():
                error_msg = "MuJoCo visualizer setup failed"
                logger.error(error_msg)
                setup_errors.append(error_msg)
                self.visualizer = None
            else:
                startup_pose_applied = False
                if isinstance(startup_joint_pose_deg, dict) and startup_joint_pose_deg:
                    l_pose = startup_joint_pose_deg.get("left")
                    r_pose = startup_joint_pose_deg.get("right")
                    if l_pose is None:
                        l_pose = self.visualizer.get_joint_angles_deg("left")
                    if r_pose is None:
                        r_pose = self.visualizer.get_joint_angles_deg("right")
                    if l_pose is not None and r_pose is not None:
                        self.visualizer.set_initial_arm_pose_deg(l_pose, r_pose)
                        self._seed_robot_state_from_pose_deg(l_pose, r_pose)
                        startup_pose_applied = True
                        logger.info(
                            "Applied startup joint_state pose to MuJoCo: "
                            f"left={np.round(l_pose, 2)}, right={np.round(r_pose, 2)}"
                        )

                # Fallback: keep unified-base default posture if no startup feedback pose.
                if use_unified_base_scene and not startup_pose_applied:
                    # 左右臂初始角互为相反数。
                    right_initial_pose_deg = np.array([-90.0, -30.0, -90.0, -90.0, -90.0, 30.0], dtype=float)
                    left_initial_pose_deg = np.array([90.0, 30.0, 90.0, 90.0, 90.0, 150.0], dtype=float)
                    self.visualizer.set_initial_arm_pose_deg(left_initial_pose_deg, right_initial_pose_deg)
                    logger.info(
                        "Applied unified-base initial pose: "
                        "left=[90, 30, 90, 90, 90, 0] deg, right=[-90, -30, -90, -90, -90, 0] deg"
                    )

                # Match physical progression to control period.
                # Example scenes use timestep=0.001s while control loop is 200Hz (0.005s),
                # so we need ~5 physics steps per control tick for responsive motion.
                try:
                    model_dt = float(self.visualizer.model.opt.timestep)
                    if model_dt > 0.0:
                        self.sim_substeps = max(1, int(round(CONTROL_DT / model_dt)))
                    else:
                        self.sim_substeps = 1
                except Exception:
                    self.sim_substeps = 1

                logger.info(
                    f"MuJoCo stepping config: control_dt={CONTROL_DT:.4f}s, "
                    f"sim_substeps={self.sim_substeps}"
                )

                # Setup Mink kinematics if enabled in config.
                if self.config.use_mink:
                    self.robot_interface.setup_mink_kinematics(
                        str(kinematics_scene_path),
                        end_effector_site
                    )
                    logger.info(f"✓ Mink+MuJoCo setup complete")
                else:
                    logger.info("Mink IK disabled by config: control.use_mink=false")
        except Exception as e:
            error_msg = f"MuJoCo visualizer setup failed with exception: {e}"
            logger.error(error_msg)
            setup_errors.append(error_msg)
            self.visualizer = None
        
        # Report all setup issues
        if setup_errors:
            logger.error("Setup failed with the following errors:")
            for i, error in enumerate(setup_errors, 1):
                logger.error(f"  {i}. {error}")
        
        # Pass robot interface to external input handler if present.
        if self.web_keyboard_handler and self.robot_interface:
            self.web_keyboard_handler.set_robot_interface(self.robot_interface)
            logger.info("Set robot interface on web keyboard handler")

        logger.info(f"Setup completed with success={success}")
        return success
    
    async def start(self):
        """Start the control loop."""
        if not self.setup():
            logger.error("Control loop setup failed")
            return
        
        self.is_running = True
        logger.info("Control loop started")
        
        # Initialize arm states with current robot positions
        self._initialize_arm_states()
        
        # Main control loop
        next_tick = time.perf_counter()
        while self.is_running:
            try:
                # Process command queue
                await self._process_commands()

                # Unified control architecture:
                # Input layer only updates goals; simulation layer executes full
                # marker -> IK -> joint target -> torque sim in one loop tick.
                if self.visualizer and self.config.use_mink:
                    self._simulation_tick_unified()
                else:
                    # Fallback for non-sim/legacy paths.
                    self._update_robot_safely()
                    if self.visualizer:
                        self._update_visualization()
                
                # Periodic logging
                self._tick_counter += 1
                self._periodic_logging()
                
                # Control rate (RateLimiter-like, monotonic clock)
                next_tick += CONTROL_DT
                sleep_s = next_tick - time.perf_counter()
                if sleep_s > 0:
                    await asyncio.sleep(sleep_s)
                else:
                    # Overrun: resync to avoid accumulating lag.
                    next_tick = time.perf_counter()
                    # Important: always yield once so VR websocket tasks can run.
                    # Without this, sustained overruns can starve input handling.
                    await asyncio.sleep(0)
                
            except Exception as e:
                logger.error(f"Error in control loop: {e}")
                await asyncio.sleep(0.1)
        
        logger.info("Control loop stopped")
    
    async def stop(self):
        """Stop the control loop."""
        self.is_running = False

        # Cleanup - disengage robot first (returns to home and disables torque)
        if self.robot_interface:
            if self.robot_interface.is_engaged:
                logger.info("🛑 Disengaging robot before shutdown...")
                self.robot_interface.disengage()
            self.robot_interface.disconnect()

        if self.visualizer:
            self.visualizer.disconnect()
    
    def _initialize_arm_states(self):
        """Initialize arm states with current robot positions."""
        if self.robot_interface:
            # In simulation/no-feedback mode, initialize command state from MuJoCo qpos
            # so IK starts from the actual simulated posture instead of zeros.
            if self.visualizer and not self.config.require_state_feedback:
                l_sim = self.visualizer.get_joint_angles_deg("left")
                r_sim = self.visualizer.get_joint_angles_deg("right")
                if l_sim is not None:
                    self.robot_interface.left_arm_angles = l_sim.copy()
                    self.robot_interface.set_simulated_arm_angles("left", l_sim)
                if r_sim is not None:
                    self.robot_interface.right_arm_angles = r_sim.copy()
                    self.robot_interface.set_simulated_arm_angles("right", r_sim)

            # Get current end effector positions
            left_pos = self._get_current_ee_position("left")
            right_pos = self._get_current_ee_position("right")
            
            # Initialize target positions to current positions (ensure deep copies)
            self.left_arm.target_position = left_pos.copy()
            self.left_arm.goal_position = left_pos.copy()
            self.left_arm.target_orientation_quat = self._get_current_ee_orientation("left")
            self.left_arm.origin_target_orientation_quat = self.left_arm.target_orientation_quat.copy()
            self.left_arm.initial_target_orientation_quat = self.left_arm.target_orientation_quat.copy()
            self.right_arm.target_position = right_pos.copy()
            self.right_arm.goal_position = right_pos.copy()
            self.right_arm.target_orientation_quat = self._get_current_ee_orientation("right")
            self.right_arm.origin_target_orientation_quat = self.right_arm.target_orientation_quat.copy()
            self.right_arm.initial_target_orientation_quat = self.right_arm.target_orientation_quat.copy()
            
            # Get current wrist roll angles
            left_angles = self.robot_interface.get_arm_angles("left")
            right_angles = self.robot_interface.get_arm_angles("right")
            
            self.left_arm.current_wrist_roll = left_angles[WRIST_ROLL_INDEX]
            self.right_arm.current_wrist_roll = right_angles[WRIST_ROLL_INDEX]
            
            self.left_arm.current_wrist_flex = left_angles[WRIST_FLEX_INDEX]
            self.right_arm.current_wrist_flex = right_angles[WRIST_FLEX_INDEX]

            # Always align simulation mocap targets with current tools_link at startup.
            if self.visualizer:
                # Keep mocap target pose fully aligned with current tools_link
                # (both position and orientation), similar to mink example behavior.
                self.visualizer.update_marker_position(
                    "left_target", left_pos, self.left_arm.target_orientation_quat
                )
                self.visualizer.update_marker_position(
                    "right_target", right_pos, self.right_arm.target_orientation_quat
                )
                # Optional goal markers follow initial target too.
                self.visualizer.update_marker_position("left_goal", left_pos)
                self.visualizer.update_marker_position("right_goal", right_pos)

                # Prevent first control ticks from being misdetected as "viewer drag changed".
                self.left_arm.last_mocap_target_position = left_pos.copy()
                self.right_arm.last_mocap_target_position = right_pos.copy()
                self.left_arm.last_mocap_target_quaternion = self.left_arm.target_orientation_quat.copy()
                self.right_arm.last_mocap_target_quaternion = self.right_arm.target_orientation_quat.copy()

                l_mocap_q = self.visualizer.get_mocap_quaternion("left_target")
                r_mocap_q = self.visualizer.get_mocap_quaternion("right_target")
                if l_mocap_q is not None:
                    l_err = self._quat_angle_error_deg_wxyz(
                        self.left_arm.target_orientation_quat, l_mocap_q
                    )
                    logger.info(
                        f"Init frame alignment LEFT tools_link vs left_target: {l_err:.3f} deg"
                    )
                if r_mocap_q is not None:
                    r_err = self._quat_angle_error_deg_wxyz(
                        self.right_arm.target_orientation_quat, r_mocap_q
                    )
                    logger.info(
                        f"Init frame alignment RIGHT tools_link vs right_target: {r_err:.3f} deg"
                    )
            
            logger.info(f"Initialized left arm at position: {left_pos.round(3)}")
            logger.info(f"Initialized right arm at position: {right_pos.round(3)}")

    def _get_current_ee_position(self, arm: str) -> np.ndarray:
        """Get current end-effector position, preferring simulation state."""
        if self.visualizer:
            sim_pos = self.visualizer.get_end_effector_position(arm)
            if sim_pos is not None:
                return sim_pos
        return self.robot_interface.get_current_end_effector_position(arm)

    def _get_current_ee_orientation(self, arm: str) -> np.ndarray:
        """Get current end-effector orientation quaternion [w, x, y, z]."""
        if self.visualizer:
            sim_quat = self.visualizer.get_end_effector_quaternion(arm)
            if sim_quat is not None:
                q = np.asarray(sim_quat, dtype=float).reshape(-1)
                if q.size >= 4:
                    q = q[:4]
                    n = np.linalg.norm(q)
                    if n > 1e-9:
                        return q / n
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

    @staticmethod
    def _quat_multiply_wxyz(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """Quaternion multiply in [w, x, y, z] convention: q = q1 ⊗ q2."""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
        ], dtype=float)

    @staticmethod
    def _quat_from_rotation_matrix_wxyz(R: np.ndarray) -> np.ndarray:
        """Convert 3x3 rotation matrix to quaternion [w, x, y, z]."""
        m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
        m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
        m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]
        tr = m00 + m11 + m22
        if tr > 0.0:
            s = np.sqrt(tr + 1.0) * 2.0
            w = 0.25 * s
            x = (m21 - m12) / s
            y = (m02 - m20) / s
            z = (m10 - m01) / s
        elif (m00 > m11) and (m00 > m22):
            s = np.sqrt(1.0 + m00 - m11 - m22) * 2.0
            w = (m21 - m12) / s
            x = 0.25 * s
            y = (m01 + m10) / s
            z = (m02 + m20) / s
        elif m11 > m22:
            s = np.sqrt(1.0 + m11 - m00 - m22) * 2.0
            w = (m02 - m20) / s
            x = (m01 + m10) / s
            y = 0.25 * s
            z = (m12 + m21) / s
        else:
            s = np.sqrt(1.0 + m22 - m00 - m11) * 2.0
            w = (m10 - m01) / s
            x = (m02 + m20) / s
            y = (m12 + m21) / s
            z = 0.25 * s
        return ControlLoop._quat_normalize_wxyz(np.array([w, x, y, z], dtype=float))

    @staticmethod
    def _quat_to_rotation_matrix_wxyz(q: np.ndarray) -> np.ndarray:
        """Convert quaternion [w, x, y, z] to 3x3 rotation matrix."""
        qn = ControlLoop._quat_normalize_wxyz(q)
        w, x, y, z = qn
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        return np.array(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ],
            dtype=float,
        )

    @staticmethod
    def _quat_from_euler_xyz_deg(rx_deg: float, ry_deg: float, rz_deg: float) -> np.ndarray:
        """Build XYZ-order rotation matrix from Euler angles (deg), then convert to [w,x,y,z]."""
        rx = np.deg2rad(rx_deg)
        ry = np.deg2rad(ry_deg)
        rz = np.deg2rad(rz_deg)
        cx, sx = np.cos(rx), np.sin(rx)
        cy, sy = np.cos(ry), np.sin(ry)
        cz, sz = np.cos(rz), np.sin(rz)
        Rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]], dtype=float)
        Ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]], dtype=float)
        Rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]], dtype=float)
        R = Rz @ Ry @ Rx
        return ControlLoop._quat_from_rotation_matrix_wxyz(R)

    @staticmethod
    def _quat_inverse_wxyz(q: np.ndarray) -> np.ndarray:
        """Quaternion inverse in [w, x, y, z] convention."""
        w, x, y, z = q
        n2 = w*w + x*x + y*y + z*z
        if n2 <= 1e-12:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        return np.array([w, -x, -y, -z], dtype=float) / n2

    @staticmethod
    def _quat_normalize_wxyz(q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=float).reshape(-1)[:4]
        n = np.linalg.norm(q)
        if n <= 1e-12:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        return q / n

    @staticmethod
    def _quat_angle_error_deg_wxyz(q_ref: np.ndarray, q_meas: np.ndarray) -> float:
        """Shortest orientation error angle between two quaternions [w, x, y, z]."""
        q_ref = np.asarray(q_ref, dtype=float).reshape(-1)[:4]
        q_meas = np.asarray(q_meas, dtype=float).reshape(-1)[:4]
        nr = np.linalg.norm(q_ref)
        nm = np.linalg.norm(q_meas)
        if nr <= 1e-12 or nm <= 1e-12:
            return 180.0
        q_ref = q_ref / nr
        q_meas = q_meas / nm
        dot = float(np.clip(abs(np.dot(q_ref, q_meas)), 0.0, 1.0))
        return float(np.degrees(2.0 * np.arccos(dot)))

    def _rotate_vec_by_quat_wxyz(self, vec: np.ndarray, quat_wxyz: np.ndarray) -> np.ndarray:
        """Rotate 3D vector by quaternion [w, x, y, z]."""
        v = np.asarray(vec, dtype=float).reshape(-1)[:3]
        q = self._quat_normalize_wxyz(quat_wxyz)
        vq = np.array([0.0, v[0], v[1], v[2]], dtype=float)
        qv = self._quat_multiply_wxyz(q, vq)
        qvq = self._quat_multiply_wxyz(qv, self._quat_inverse_wxyz(q))
        return qvq[1:4].copy()
    
    async def _process_commands(self):
        """Process commands from the command queue."""
        try:
            # Process regular control goals
            while not self.command_queue.empty():
                goal = self.command_queue.get_nowait()
                await self._execute_goal(goal)
        except Exception as e:
            logger.error(f"Error processing commands: {e}")
            import traceback
            logger.error(f"Traceback: {traceback.format_exc()}")
    
    async def _handle_command(self, command):
        """Handle individual commands."""
        action = command.get('action', '')
        logger.info(f"🔌 Processing control command: {action}")

        if action in {'enable_keyboard', 'disable_keyboard', 'web_keypress'}:
            logger.warning(f"🎮 Keyboard control has been removed, ignoring command: {action}")
            return
        elif action == 'robot_connect':
            logger.info("🔌 Processing robot_connect command")
            if self.robot_interface and self.robot_interface.is_connected:
                logger.info(f"🔌 Robot interface available and connected: {self.robot_interface.is_connected}")
                success = self.robot_interface.engage()
                if success:
                    logger.info("🔌 Robot motors ENGAGED via API")
                    # Unified control path keeps targets synchronized automatically.
                else:
                    logger.error("❌ Failed to engage robot motors")
            else:
                logger.warning(f"Cannot engage robot: interface={self.robot_interface is not None}, connected={self.robot_interface.is_connected if self.robot_interface else False}")
        elif action == 'robot_disconnect':
            logger.info("🔌 Processing robot_disconnect command")
            if self.robot_interface:
                logger.info(f"🔌 Robot interface available")
                success = self.robot_interface.disengage()
                if success:
                    logger.info("🔌 Robot motors DISENGAGED via API")
                    # Reset arm states to IDLE when robot is disengaged
                    self.left_arm.reset()
                    self.right_arm.reset()
                    logger.info("🔓 Both arms: Position control DEACTIVATED after robot disconnect")
                    
                    # Hide visualization markers
                    if self.visualizer:
                        for arm in ["left", "right"]:
                            self.visualizer.hide_marker(f"{arm}_goal")
                            self.visualizer.hide_frame(f"{arm}_goal_frame")
                            self.visualizer.hide_marker(f"{arm}_target")
                            self.visualizer.hide_frame(f"{arm}_target_frame")
                else:
                    logger.error("❌ Failed to disengage robot motors")
            else:
                logger.warning("Cannot disengage robot: no robot interface")
        else:
            logger.warning(f"Unknown command: {action}")

    async def _execute_goal(self, goal: ControlGoal):
        """Execute a control goal."""
        arm_state = self.left_arm if goal.arm == "left" else self.right_arm
        
        # Handle special reset signal from external input timeout.
        if (goal.metadata and goal.metadata.get("reset_target_to_current", False)):
            if self.robot_interface and arm_state.mode == ControlMode.POSITION_CONTROL:
                # Reset target position to current robot position
                current_position = self._get_current_ee_position(goal.arm)
                current_angles = self.robot_interface.get_arm_angles(goal.arm)
                
                arm_state.target_position = current_position.copy()
                arm_state.goal_position = current_position.copy()
                arm_state.target_orientation_quat = self._get_current_ee_orientation(goal.arm)
                arm_state.origin_target_orientation_quat = arm_state.target_orientation_quat.copy()
                arm_state.origin_position = current_position.copy()
                arm_state.current_wrist_roll = current_angles[WRIST_ROLL_INDEX]
                arm_state.current_wrist_flex = current_angles[WRIST_FLEX_INDEX]
                arm_state.origin_wrist_roll_angle = current_angles[WRIST_ROLL_INDEX]
                arm_state.origin_wrist_flex_angle = current_angles[WRIST_FLEX_INDEX]
                arm_state.controller_to_target_offset_quat = None
                
                logger.info(f"🔄 {goal.arm.upper()} arm: Target position reset to current robot position (idle timeout)")
            return
        
        # Handle mode changes (only if mode is specified)
        if goal.mode is not None and goal.mode != arm_state.mode:
            if goal.mode == ControlMode.POSITION_CONTROL:
                # Activate position control - always reset target to current position
                arm_state.mode = ControlMode.POSITION_CONTROL
                
                if self.robot_interface:
                    current_position = self._get_current_ee_position(goal.arm)
                    current_angles = self.robot_interface.get_arm_angles(goal.arm)
                    
                    # Reset everything to current position (like VR grip press)
                    arm_state.target_position = current_position.copy()
                    arm_state.goal_position = current_position.copy()
                    arm_state.target_orientation_quat = self._get_current_ee_orientation(goal.arm)
                    arm_state.origin_target_orientation_quat = arm_state.target_orientation_quat.copy()
                    arm_state.origin_position = current_position.copy()
                    arm_state.current_wrist_roll = current_angles[WRIST_ROLL_INDEX]
                    arm_state.current_wrist_flex = current_angles[WRIST_FLEX_INDEX]
                    arm_state.origin_wrist_roll_angle = current_angles[WRIST_ROLL_INDEX]
                    arm_state.origin_wrist_flex_angle = current_angles[WRIST_FLEX_INDEX]
                    arm_state.controller_to_target_offset_quat = None
                
                logger.info(f"🔒 {goal.arm.upper()} arm: Position control ACTIVATED (target reset to current position)")
                
            elif goal.mode == ControlMode.IDLE:
                # Deactivate position control without introducing a one-tick mocap jump.
                # Keep target marker aligned to current EE so release is stable.
                current_position = self._get_current_ee_position(goal.arm)
                current_quat = self._get_current_ee_orientation(goal.arm)

                arm_state.mode = ControlMode.IDLE
                arm_state.target_position = None
                arm_state.target_orientation_quat = None
                arm_state.origin_target_orientation_quat = None
                arm_state.goal_position = None
                arm_state.origin_position = None
                arm_state.origin_wrist_roll_angle = 0.0
                arm_state.origin_wrist_flex_angle = 0.0
                arm_state.controller_to_target_offset_quat = None

                # Seed mocap history to avoid false "drag changed" detection immediately after release.
                arm_state.last_mocap_target_position = current_position.copy()
                arm_state.last_mocap_target_quaternion = current_quat.copy()

                # Keep target marker at current EE pose when leaving grab mode.
                if self.visualizer:
                    self.visualizer.update_marker_position(
                        f"{goal.arm}_target", current_position, current_quat
                    )
                    self.visualizer.hide_marker(f"{goal.arm}_goal")
                    self.visualizer.hide_frame(f"{goal.arm}_goal_frame")
                
                logger.info(f"🔓 {goal.arm.upper()} arm: Position control DEACTIVATED")
        
        # Position control uses absolute offset from origin.
        if goal.target_position is not None and arm_state.mode == ControlMode.POSITION_CONTROL:
            if goal.metadata and goal.metadata.get("relative_position", False):
                delta = np.asarray(goal.target_position, dtype=float)
                delta_base = self._rotate_vec_by_quat_wxyz(delta, self._vr_delta_to_base_quat)
                # Relative input is interpreted as absolute offset from arm origin.
                if arm_state.origin_position is not None:
                    arm_state.target_position = arm_state.origin_position + delta_base
                    arm_state.goal_position = arm_state.target_position.copy()
                else:
                    # No origin set yet, use current position as base
                    if self.robot_interface:
                        current_position = self._get_current_ee_position(goal.arm)
                        arm_state.target_position = current_position + delta_base
                        arm_state.goal_position = arm_state.target_position.copy()
            else:
                # Absolute position (legacy - should not be used anymore)
                arm_state.target_position = goal.target_position.copy()
                arm_state.goal_position = goal.target_position.copy()

            if goal.target_orientation_quat is not None:
                q = np.asarray(goal.target_orientation_quat, dtype=float).reshape(-1)
                if q.size >= 4:
                    q_in = self._quat_normalize_wxyz(q[:4])
                    q_mapped = self._quat_normalize_wxyz(q_in)
                    use_marker_grab_drag = bool(
                        goal.metadata and goal.metadata.get("marker_grab_drag", False)
                    )

                    if use_marker_grab_drag:
                        # Grip-drag mode: input quaternion is controller-local delta
                        # relative to grip press. Convert that delta into target frame,
                        # then apply it on the grip-time target orientation.
                        R_delta_hand = self._quat_to_rotation_matrix_wxyz(q_mapped)
                        S = self._controller_delta_to_target_axis_map
                        R_delta_target = S @ R_delta_hand @ S.T
                        U = self._relative_rotation_post_axis_map
                        R_delta_target = U @ R_delta_target @ U.T
                        q_delta_target = self._quat_from_rotation_matrix_wxyz(
                            R_delta_target
                        )
                        q_init = arm_state.origin_target_orientation_quat
                        if q_init is None:
                            q_init = self._get_current_ee_orientation(goal.arm)
                        q_init = self._quat_normalize_wxyz(np.asarray(q_init, dtype=float))
                        q_out = self._quat_multiply_wxyz(q_init, q_delta_target)
                        arm_state.target_orientation_quat = self._quat_normalize_wxyz(q_out)
                    else:
                        # Fallback absolute quaternion path.
                        arm_state.target_orientation_quat = q_mapped
            
            # Handle wrist roll using absolute offset from origin.
            if goal.wrist_roll_deg is not None:
                if goal.metadata and goal.metadata.get("relative_position", False):
                    # Relative input is interpreted from grip-time wrist origin.
                    arm_state.current_wrist_roll = arm_state.origin_wrist_roll_angle + goal.wrist_roll_deg
                else:
                    # Absolute wrist roll (legacy)
                    arm_state.current_wrist_roll = goal.wrist_roll_deg
            
            # Handle wrist flex using absolute offset from origin.
            if goal.wrist_flex_deg is not None:
                if goal.metadata and goal.metadata.get("relative_position", False):
                    # Relative input is interpreted from grip-time wrist origin.
                    arm_state.current_wrist_flex = arm_state.origin_wrist_flex_angle + goal.wrist_flex_deg
                else:
                    # Absolute wrist flex (legacy)
                    arm_state.current_wrist_flex = goal.wrist_flex_deg
        
        # Handle gripper control (independent of mode)
        if goal.gripper_closed is not None and self.robot_interface:
            self.robot_interface.set_gripper(goal.arm, goal.gripper_closed)
            if self.visualizer:
                self.visualizer.set_gripper_closed(goal.arm, goal.gripper_closed)
    
    def _update_robot_safely(self):
        """Update robot with current control goals (with error handling)."""
        if not self.robot_interface:
            return
        
        try:
            self._update_robot()
        except Exception as e:
            logger.error(f"Error updating robot: {e}")
            # Don't shutdown, just continue - robot interface will handle connection issues

    def _simulation_tick_unified(self):
        """Single-pass simulation tick: marker -> IK -> command -> sim step.

        This mirrors the architecture of mink example scripts more closely by
        keeping all control actions in one deterministic loop.
        """
        if not self.robot_interface or not self.visualizer:
            return

        ik_requests = {
            "left": None,
            "right": None,
        }

        for arm_name, arm_state in (("left", self.left_arm), ("right", self.right_arm)):
            target_name = f"{arm_name}_target"
            goal_name = f"{arm_name}_goal"

            # 1) Resolve IK target source (input stream marker or viewer drag marker).
            drag_target = self.visualizer.get_mocap_position(target_name)
            drag_quat = self.visualizer.get_mocap_quaternion(target_name)
            drag_active = False
            if arm_state.mode == ControlMode.IDLE and drag_target is not None:
                prev_p = arm_state.last_mocap_target_position
                prev_q = arm_state.last_mocap_target_quaternion
                pos_changed = prev_p is None or np.linalg.norm(drag_target - prev_p) > 1e-5
                quat_changed = False
                if drag_quat is not None:
                    q = np.asarray(drag_quat, dtype=float)
                    n = np.linalg.norm(q)
                    if n > 1e-9:
                        q = q / n
                        if prev_q is None:
                            quat_changed = True
                        else:
                            # Account for quaternion sign ambiguity (q and -q are same rotation)
                            quat_changed = min(np.linalg.norm(q - prev_q), np.linalg.norm(q + prev_q)) > 1e-4
                        drag_quat = q
                drag_active = pos_changed or quat_changed

            should_solve = (
                (arm_state.mode == ControlMode.POSITION_CONTROL and arm_state.target_position is not None)
                or drag_active
            )

            if should_solve:
                ik_target = arm_state.target_position if arm_state.target_position is not None else drag_target
                ik_target_quat = arm_state.target_orientation_quat

                # Position-control mode commands the marker from input goals.
                if arm_state.mode == ControlMode.POSITION_CONTROL:
                    self.visualizer.update_marker_position(target_name, ik_target, arm_state.target_orientation_quat)

                mocap_pos = self.visualizer.get_mocap_position(target_name)
                if mocap_pos is not None:
                    ik_target = mocap_pos
                    arm_state.last_mocap_target_position = mocap_pos.copy()
                mocap_quat = self.visualizer.get_mocap_quaternion(target_name)
                if mocap_quat is not None:
                    q = np.asarray(mocap_quat, dtype=float)
                    n = np.linalg.norm(q)
                    if n > 1e-9:
                        q = q / n
                        ik_target_quat = q
                        arm_state.last_mocap_target_quaternion = q.copy()

                # 2) Legacy smoothing / target shaping before IK.
                current_ee = self._get_current_ee_position(arm_name)
                ik_target = current_ee + IK_TARGET_SMOOTHING * (ik_target - current_ee)

                # 3) 收集双臂IK请求，统一交给共享Mink求解器。
                ik_requests[arm_name] = {
                    "target": ik_target,
                    "target_quat": ik_target_quat,
                    "wrist_override": (arm_state.target_orientation_quat is None),
                }

            # 4) Marker/goal visibility policy.
            if arm_state.mode == ControlMode.POSITION_CONTROL:
                if arm_state.target_position is not None:
                    self.visualizer.update_marker_position(target_name, arm_state.target_position, arm_state.target_orientation_quat)
                    self.visualizer.update_coordinate_frame(f"{target_name}_frame", arm_state.target_position)
                if arm_state.goal_position is not None:
                    self.visualizer.update_marker_position(goal_name, arm_state.goal_position)
                    self.visualizer.update_coordinate_frame(f"{goal_name}_frame", arm_state.goal_position)
            else:
                self.visualizer.hide_marker(goal_name)
                self.visualizer.hide_frame(f"{target_name}_frame")
                self.visualizer.hide_frame(f"{goal_name}_frame")

            # Always show current tools_link pose as an "actual" marker.
            actual_pos = self._get_current_ee_position(arm_name)
            actual_quat = self._get_current_ee_orientation(arm_name)
            self.visualizer.update_marker_position(f"{arm_name}_actual", actual_pos, actual_quat)

        # 4.5) 双臂一次联合IK求解（与 arm_arm620_dual 示例一致）。
        if ik_requests["left"] is not None or ik_requests["right"] is not None:
            left_req = ik_requests["left"]
            right_req = ik_requests["right"]

            left_target = left_req["target"] if left_req is not None else self._get_current_ee_position("left")
            right_target = right_req["target"] if right_req is not None else self._get_current_ee_position("right")
            left_target_quat = left_req["target_quat"] if left_req is not None else None
            right_target_quat = right_req["target_quat"] if right_req is not None else None

            self._mink_solve_counter += 1
            left_solution, right_solution = self.robot_interface.solve_dual_ik(
                left_target_position=left_target,
                right_target_position=right_target,
                left_target_orientation=left_target_quat,
                right_target_orientation=right_target_quat,
            )

            if left_req is not None:
                left_gripper = self.robot_interface.get_arm_angles("left")[GRIPPER_INDEX]
                self.robot_interface.update_arm_angles(
                    "left",
                    left_solution,
                    self.left_arm.current_wrist_flex,
                    self.left_arm.current_wrist_roll,
                    left_gripper,
                    wrist_override=left_req["wrist_override"],
                )

            if right_req is not None:
                right_gripper = self.robot_interface.get_arm_angles("right")[GRIPPER_INDEX]
                self.robot_interface.update_arm_angles(
                    "right",
                    right_solution,
                    self.right_arm.current_wrist_flex,
                    self.right_arm.current_wrist_roll,
                    right_gripper,
                    wrist_override=right_req["wrist_override"],
                )

        # 5) Push commanded targets into simulation + step once.
        if self.config.require_state_feedback:
            l_ang = self.robot_interface.get_actual_arm_angles("left")
            r_ang = self.robot_interface.get_actual_arm_angles("right")
        else:
            l_ang = self.robot_interface.get_arm_angles("left")
            r_ang = self.robot_interface.get_arm_angles("right")

        self.visualizer.update_robot_pose(l_ang, "left")
        self.visualizer.update_robot_pose(r_ang, "right")
        self.visualizer.set_gripper_closed("left", self.robot_interface.get_gripper_closed("left"))
        self.visualizer.set_gripper_closed("right", self.robot_interface.get_gripper_closed("right"))
        self._mujoco_substep_counter += int(self.sim_substeps)
        self.visualizer.step_simulation(substeps=self.sim_substeps)

        # 6) Feed simulated feedback in no-feedback mode.
        if not self.config.require_state_feedback:
            l_sim = self.visualizer.get_joint_angles_deg("left")
            r_sim = self.visualizer.get_joint_angles_deg("right")
            if l_sim is not None:
                self.robot_interface.set_simulated_arm_angles("left", l_sim)
            if r_sim is not None:
                self.robot_interface.set_simulated_arm_angles("right", r_sim)

        # 7) Publish hardware commands (if enabled/engaged).
        if self.robot_interface.is_connected and self.robot_interface.is_engaged:
            self.robot_interface.send_command()
    
    def _update_robot(self):
        """Update robot with current control goals."""
        if not self.robot_interface:
            return

        if not self.config.use_mink:
            # Keep gripper commands and passthrough robot publishing only.
            if self.robot_interface.is_connected and self.robot_interface.is_engaged:
                self.robot_interface.send_command()
            return

        ik_requests = {
            "left": None,
            "right": None,
        }

        for arm_name, arm_state in (("left", self.left_arm), ("right", self.right_arm)):
            target_name = f"{arm_name}_target"

            drag_target = self.visualizer.get_mocap_position(target_name) if self.visualizer else None

            drag_active = False
            if arm_state.mode == ControlMode.IDLE and drag_target is not None:
                prev = arm_state.last_mocap_target_position
                drag_active = prev is None or np.linalg.norm(drag_target - prev) > 1e-5

            should_solve = (
                (arm_state.mode == ControlMode.POSITION_CONTROL and arm_state.target_position is not None)
                or drag_active
            )

            if not should_solve:
                continue

            ik_target = arm_state.target_position if arm_state.target_position is not None else drag_target
            ik_target_quat = arm_state.target_orientation_quat

            if self.visualizer:
                if arm_state.mode == ControlMode.POSITION_CONTROL:
                    self.visualizer.update_marker_position(
                        target_name, ik_target, arm_state.target_orientation_quat
                    )
                mocap_pos = self.visualizer.get_mocap_position(target_name)
                if mocap_pos is not None:
                    ik_target = mocap_pos
                    arm_state.last_mocap_target_position = mocap_pos.copy()
                mocap_quat = self.visualizer.get_mocap_quaternion(target_name)
                if mocap_quat is not None:
                    q = np.asarray(mocap_quat, dtype=float)
                    n = np.linalg.norm(q)
                    if n > 1e-9:
                        ik_target_quat = q / n

            current_ee = self._get_current_ee_position(arm_name)
            ik_target = current_ee + IK_TARGET_SMOOTHING * (ik_target - current_ee)

            ik_requests[arm_name] = {
                "target": ik_target,
                "target_quat": ik_target_quat,
                "wrist_override": True,
            }

        if ik_requests["left"] is not None or ik_requests["right"] is not None:
            left_req = ik_requests["left"]
            right_req = ik_requests["right"]

            left_target = left_req["target"] if left_req is not None else self._get_current_ee_position("left")
            right_target = right_req["target"] if right_req is not None else self._get_current_ee_position("right")
            left_target_quat = left_req["target_quat"] if left_req is not None else None
            right_target_quat = right_req["target_quat"] if right_req is not None else None

            self._mink_solve_counter += 1
            left_solution, right_solution = self.robot_interface.solve_dual_ik(
                left_target_position=left_target,
                right_target_position=right_target,
                left_target_orientation=left_target_quat,
                right_target_orientation=right_target_quat,
            )

            if left_req is not None:
                left_gripper = self.robot_interface.get_arm_angles("left")[GRIPPER_INDEX]
                self.robot_interface.update_arm_angles(
                    "left",
                    left_solution,
                    self.left_arm.current_wrist_flex,
                    self.left_arm.current_wrist_roll,
                    left_gripper,
                    wrist_override=left_req["wrist_override"],
                )

            if right_req is not None:
                right_gripper = self.robot_interface.get_arm_angles("right")[GRIPPER_INDEX]
                self.robot_interface.update_arm_angles(
                    "right",
                    right_solution,
                    self.right_arm.current_wrist_flex,
                    self.right_arm.current_wrist_roll,
                    right_gripper,
                    wrist_override=right_req["wrist_override"],
                )

        # Send commands to robot hardware (only if connected and engaged)
        if self.robot_interface.is_connected and self.robot_interface.is_engaged:
            self.robot_interface.send_command()

    def _update_visualization(self):
        """Update MuJoCo visualization."""
        if not self.visualizer:
            return
        
        # In no-feedback/simulation mode, drive the visualizer from commanded angles.
        # Otherwise use hardware feedback angles.
        if self.config.require_state_feedback:
            left_angles = self.robot_interface.get_actual_arm_angles("left")
            right_angles = self.robot_interface.get_actual_arm_angles("right")
        else:
            left_angles = self.robot_interface.get_arm_angles("left")
            right_angles = self.robot_interface.get_arm_angles("right")
        
        self.visualizer.update_robot_pose(left_angles, 'left')
        self.visualizer.update_robot_pose(right_angles, 'right')
        self.visualizer.set_gripper_closed("left", self.robot_interface.get_gripper_closed("left"))
        self.visualizer.set_gripper_closed("right", self.robot_interface.get_gripper_closed("right"))
        
        # Update visualization markers
        if self.left_arm.mode == ControlMode.POSITION_CONTROL:
            if self.left_arm.target_position is not None:
                # Show controller-provided target position (desired EE target)
                target_pos = self.left_arm.target_position
                self.visualizer.update_marker_position(
                    "left_target", target_pos, self.left_arm.target_orientation_quat
                )
                self.visualizer.update_coordinate_frame("left_target_frame", target_pos)
            
            if self.left_arm.goal_position is not None:
                # Show goal position
                self.visualizer.update_marker_position("left_goal", self.left_arm.goal_position)
                self.visualizer.update_coordinate_frame("left_goal_frame", self.left_arm.goal_position)
        else:
            # In idle mode, keep target marker draggable by user (do not overwrite it).
            # Hide only goal marker when not in active dragging.
            self.visualizer.hide_marker("left_goal")
            self.visualizer.hide_frame("left_target_frame")
            self.visualizer.hide_frame("left_goal_frame")
        
        if self.right_arm.mode == ControlMode.POSITION_CONTROL:
            if self.right_arm.target_position is not None:
                # Show controller-provided target position (desired EE target)
                target_pos = self.right_arm.target_position
                self.visualizer.update_marker_position(
                    "right_target", target_pos, self.right_arm.target_orientation_quat
                )
                self.visualizer.update_coordinate_frame("right_target_frame", target_pos)
            
            if self.right_arm.goal_position is not None:
                # Show goal position
                self.visualizer.update_marker_position("right_goal", self.right_arm.goal_position)
                self.visualizer.update_coordinate_frame("right_goal_frame", self.right_arm.goal_position)
        else:
            # In idle mode, keep target marker draggable by user (do not overwrite it).
            # Hide only goal marker when not in active dragging.
            self.visualizer.hide_marker("right_goal")
            self.visualizer.hide_frame("right_target_frame")
            self.visualizer.hide_frame("right_goal_frame")
        
        # Step simulation once per control tick (same pattern as arm_arm620_dual).
        self._mujoco_substep_counter += int(self.sim_substeps)
        self.visualizer.step_simulation(substeps=self.sim_substeps)

        # Feed back simulated joint angles when ROS state feedback is disabled.
        if not self.config.require_state_feedback:
            l_sim = self.visualizer.get_joint_angles_deg("left")
            r_sim = self.visualizer.get_joint_angles_deg("right")
            if l_sim is not None:
                self.robot_interface.set_simulated_arm_angles("left", l_sim)
            if r_sim is not None:
                self.robot_interface.set_simulated_arm_angles("right", r_sim)
    
    def _periodic_logging(self):
        """Log status information periodically."""
        current_time = time.time()
        if current_time - self.last_log_time >= self.log_interval:
            self.last_log_time = current_time

            elapsed = max(1e-6, time.perf_counter() - self._perf_window_start)
            ctrl_hz = self._tick_counter / elapsed
            mink_hz = self._mink_solve_counter / elapsed
            mujoco_substep_hz = self._mujoco_substep_counter / elapsed
            model_dt = None
            if self.visualizer and getattr(self.visualizer, 'model', None) is not None:
                try:
                    model_dt = float(self.visualizer.model.opt.timestep)
                except Exception:
                    model_dt = None

            if model_dt is not None and model_dt > 0:
                logger.debug(
                    f"⏱ runtime: control={ctrl_hz:.1f}Hz | mink_solve={mink_hz:.1f}Hz | "
                    f"mujoco_substep={mujoco_substep_hz:.1f}Hz (substeps={self.sim_substeps}, model_dt={model_dt:.4f}s)"
                )
            else:
                logger.debug(
                    f"⏱ runtime: control={ctrl_hz:.1f}Hz | mink_solve={mink_hz:.1f}Hz | "
                    f"mujoco_substep={mujoco_substep_hz:.1f}Hz (substeps={self.sim_substeps})"
                )

            self._perf_window_start = time.perf_counter()
            self._tick_counter = 0
            self._mink_solve_counter = 0
            self._mujoco_substep_counter = 0
            
            active_arms = []
            if self.left_arm.mode == ControlMode.POSITION_CONTROL:
                active_arms.append("LEFT")
            if self.right_arm.mode == ControlMode.POSITION_CONTROL:
                active_arms.append("RIGHT")
            
            if active_arms and self.robot_interface:
                left_angles = self.robot_interface.get_arm_angles("left")
                right_angles = self.robot_interface.get_arm_angles("right")
                logger.info(f"🤖 Active control: {', '.join(active_arms)} | Left: {left_angles.round(1)} | Right: {right_angles.round(1)}")
    
    @property
    def status(self) -> Dict:
        """Get current control loop status."""
        return {
            "running": self.is_running,
            "left_arm_mode": self.left_arm.mode.value,
            "right_arm_mode": self.right_arm.mode.value,
            "robot_connected": self.robot_interface.is_connected if self.robot_interface else False,
            "left_arm_connected": self.robot_interface.get_arm_connection_status("left") if self.robot_interface else False,
            "right_arm_connected": self.robot_interface.get_arm_connection_status("right") if self.robot_interface else False,
            "visualizer_connected": self.visualizer.is_connected if self.visualizer else False,
        } 
