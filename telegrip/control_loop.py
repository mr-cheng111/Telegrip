"""
Main control loop for the teleoperation system.
Consumes control goals from the command queue and executes them via the robot interface.
"""

import asyncio
import numpy as np
import logging
import time
from dataclasses import dataclass, field
from typing import Dict, Optional

from .config import TelegripConfig, NUM_JOINTS, WRIST_FLEX_INDEX, WRIST_ROLL_INDEX, GRIPPER_INDEX
from .core.robot_interface import RobotInterface
from .core.teleop_frame_mapper import TeleopFrameMapper
# MuJoCoVisualizer will be imported on demand
from .inputs.base import ControlGoal, ControlMode
# Optional input handlers are imported on demand to avoid circular imports

logger = logging.getLogger(__name__)

# Match mink/examples/arm_arm620_dual.py behavior.
CONTROL_HZ = 200.0
CONTROL_DT = 1.0 / CONTROL_HZ
IK_TARGET_SMOOTHING = 1.0
ARM_NAMES = ("left", "right")


@dataclass
class ArmRuntimeState:
    """单臂运行态。统一收拢 teleop 控制过程中所有会变化的机械臂状态。"""

    arm_name: str
    mode: ControlMode = ControlMode.IDLE
    target_position: Optional[np.ndarray] = None
    target_orientation_quat: Optional[np.ndarray] = None
    origin_target_orientation_quat: Optional[np.ndarray] = None
    goal_position: Optional[np.ndarray] = None
    origin_position: Optional[np.ndarray] = None
    origin_wrist_roll_angle: float = 0.0
    origin_wrist_flex_angle: float = 0.0
    current_wrist_roll: float = 0.0
    current_wrist_flex: float = 0.0
    last_mocap_target_position: Optional[np.ndarray] = None
    last_mocap_target_quaternion: Optional[np.ndarray] = None

    def reset(self):
        """完全重置单臂运行态。"""
        self.mode = ControlMode.IDLE
        self.target_position = None
        self.target_orientation_quat = None
        self.origin_target_orientation_quat = None
        self.goal_position = None
        self.origin_position = None
        self.origin_wrist_roll_angle = 0.0
        self.origin_wrist_flex_angle = 0.0
        self.current_wrist_roll = 0.0
        self.current_wrist_flex = 0.0
        self.last_mocap_target_position = None
        self.last_mocap_target_quaternion = None

    def clear_target_state(self):
        """清空当前位置控制相关目标，但保留当前 wrist 实时值。"""
        self.mode = ControlMode.IDLE
        self.target_position = None
        self.target_orientation_quat = None
        self.origin_target_orientation_quat = None
        self.goal_position = None
        self.origin_position = None
        self.origin_wrist_roll_angle = 0.0
        self.origin_wrist_flex_angle = 0.0

    def seed_from_pose(
        self,
        position: np.ndarray,
        quat_wxyz: np.ndarray,
        wrist_roll_deg: float,
        wrist_flex_deg: float,
    ):
        """用当前机械臂姿态重置控制目标，避免进入控制时第一帧跳变。"""
        self.target_position = np.asarray(position, dtype=float).copy()
        self.goal_position = np.asarray(position, dtype=float).copy()
        self.target_orientation_quat = np.asarray(quat_wxyz, dtype=float).copy()
        self.origin_target_orientation_quat = np.asarray(quat_wxyz, dtype=float).copy()
        self.origin_position = np.asarray(position, dtype=float).copy()
        self.current_wrist_roll = float(wrist_roll_deg)
        self.current_wrist_flex = float(wrist_flex_deg)
        self.origin_wrist_roll_angle = float(wrist_roll_deg)
        self.origin_wrist_flex_angle = float(wrist_flex_deg)

    def record_mocap(self, position: np.ndarray, quat_wxyz: np.ndarray):
        """记录当前 marker/mocap 对齐结果，供下一帧复用。"""
        self.last_mocap_target_position = np.asarray(position, dtype=float).copy()
        self.last_mocap_target_quaternion = np.asarray(quat_wxyz, dtype=float).copy()


class ControlLoop:
    """Main control loop that processes command queue and controls robot."""
    
    def __init__(self, command_queue: asyncio.Queue, config: TelegripConfig):
        self.command_queue = command_queue
        self.config = config
        
        # Components
        self.robot_interface = None
        self.visualizer = None
        
        # Arm states
        self.arm_states = {
            arm: ArmRuntimeState(arm_name=arm)
            for arm in ARM_NAMES
        }
        
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
        # 所有 teleop 相关的平移/姿态补偿统一收拢到 TeleopFrameMapper。
        relative_rotation_post_axis_map = TeleopFrameMapper.parse_axis_remap_matrix(
            getattr(self.config, "teleop_frame_relative_rotation_axis_map", [])
        )
        if not np.allclose(relative_rotation_post_axis_map, np.eye(3), atol=1e-9):
            logger.info(
                "Using teleop_frame.relative_rotation_axis_map:\n"
                f"{relative_rotation_post_axis_map}"
            )
        self.frame_mapper = TeleopFrameMapper.from_telegrip_config(self.config)
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
                logger.warning(error_msg)
                setup_errors.append(error_msg)
                logger.warning(
                    "Continuing startup in disconnected mode; "
                    "backend will auto-reconnect when hardware process is available."
                )
        except Exception as e:
            error_msg = f"Robot interface setup failed with exception: {e}"
            logger.error(error_msg)
            setup_errors.append(error_msg)
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
            
            # Kinematics scene and end-effector site are configurable.
            kinematics_scene = self.config.mink_mujoco_scene
            end_effector_site = self.config.end_effector_site
            
            kinematics_scene_path = get_absolute_path(kinematics_scene)
            scene_name = kinematics_scene_path.name.lower()
            use_unified_base_scene = "unified" in scene_name
            visualizer_scene_path = kinematics_scene_path
            logger.info(
                "Using fixed MuJoCo scene file directly for visualization and Mink IK: "
                f"{kinematics_scene_path}"
            )
            
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
                self._control_tick()
                
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

        # Cleanup - backend disconnect only (motor enable is managed externally).
        if self.robot_interface:
            self.robot_interface.disconnect()

        if self.visualizer:
            self.visualizer.disconnect()

    def _control_tick(self):
        """
        单帧控制入口。

        固定流程：
            1. 根据需要把 /joint_states 回灌到 MuJoCo
            2. 基于当前 arm_state 构建 IK 请求
            3. 执行双臂 IK 并更新命令角
            4. 更新可视化/仿真状态
            5. 发布真实机器人命令

        这样避免旧实现中 `_simulation_tick_unified`、`_update_robot`
        和 `_update_visualization` 三套流程并行维护。
        """
        if not self.robot_interface:
            return

        if self.visualizer and (
            self.config.require_state_feedback
            or bool(getattr(self.config, "require_joint_state_for_motion", False))
        ):
            self._sync_mujoco_from_joint_state()

        if self.config.use_mink and self.visualizer:
            ik_requests = {
                arm: self._build_ik_request_for_arm(arm, self.arm_states[arm])
                for arm in ARM_NAMES
            }
            self._execute_dual_ik_requests(ik_requests)

        self._update_visualizer_state()
        self._publish_robot_command()
    
    def _initialize_arm_states(self):
        """Initialize arm states with current robot positions."""
        if self.robot_interface:
            # In simulation/no-feedback mode, initialize command state from MuJoCo qpos
            # so IK starts from the actual simulated posture instead of zeros.
            if (
                self.visualizer
                and not self.config.require_state_feedback
                and not getattr(self.config, "require_joint_state_for_motion", False)
            ):
                for arm in ARM_NAMES:
                    sim_angles = self.visualizer.get_joint_angles_deg(arm)
                    if sim_angles is not None:
                        if arm == "left":
                            self.robot_interface.left_arm_angles = sim_angles.copy()
                        else:
                            self.robot_interface.right_arm_angles = sim_angles.copy()
                        self.robot_interface.set_simulated_arm_angles(arm, sim_angles)

            # Always align simulation mocap targets with current tools_link at startup.
            if self.visualizer:
                for arm in ARM_NAMES:
                    arm_state = self.arm_states[arm]
                    current_pos, current_quat, current_angles = self._capture_current_arm_pose(arm)
                    arm_state.seed_from_pose(
                        position=current_pos,
                        quat_wxyz=current_quat,
                        wrist_roll_deg=current_angles[WRIST_ROLL_INDEX],
                        wrist_flex_deg=current_angles[WRIST_FLEX_INDEX],
                    )
                    self.visualizer.update_marker_position(
                        f"{arm}_target", current_pos, arm_state.target_orientation_quat
                    )
                    self.visualizer.update_marker_position(f"{arm}_goal", current_pos)
                    arm_state.record_mocap(current_pos, arm_state.target_orientation_quat)

                    mocap_q = self.visualizer.get_mocap_quaternion(f"{arm}_target")
                    if mocap_q is not None:
                        err = self._quat_angle_error_deg_wxyz(
                            arm_state.target_orientation_quat, mocap_q
                        )
                        logger.info(
                            f"Init frame alignment {arm.upper()} tools_link vs {arm}_target: {err:.3f} deg"
                        )

                    logger.info(f"Initialized {arm} arm at position: {current_pos.round(3)}")
            else:
                for arm in ARM_NAMES:
                    arm_state = self.arm_states[arm]
                    current_pos, current_quat, current_angles = self._capture_current_arm_pose(arm)
                    arm_state.seed_from_pose(
                        position=current_pos,
                        quat_wxyz=current_quat,
                        wrist_roll_deg=current_angles[WRIST_ROLL_INDEX],
                        wrist_flex_deg=current_angles[WRIST_FLEX_INDEX],
                    )
                    logger.info(f"Initialized {arm} arm at position: {current_pos.round(3)}")

    def _get_current_ee_position(self, arm: str) -> np.ndarray:
        """Get current end-effector position, preferring simulation state."""
        if self.visualizer:
            sim_pos = self.visualizer.get_end_effector_position(arm)
            if sim_pos is not None:
                return sim_pos
        return self.robot_interface.get_current_end_effector_position(arm)

    def _sync_mujoco_from_joint_state(self):
        """将 /joint_states 反馈回灌到 MuJoCo 当前姿态。"""
        if not self.visualizer or not self.robot_interface:
            return
        if not self.robot_interface.is_motion_gate_ready():
            return

        left_feedback = self.robot_interface.get_actual_arm_angles("left")
        right_feedback = self.robot_interface.get_actual_arm_angles("right")
        synced = self.visualizer.sync_robot_pose_from_feedback(left_feedback, right_feedback)
        if synced:
            # 维护一份“仿真侧观测角”，便于无反馈分支与状态面板复用。
            self.robot_interface.set_simulated_arm_angles("left", left_feedback)
            self.robot_interface.set_simulated_arm_angles("right", right_feedback)

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
    def _quat_normalize_wxyz(q: np.ndarray) -> np.ndarray:
        return TeleopFrameMapper.normalize_wxyz(q)

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

    def _get_arm_state(self, arm: str) -> ArmRuntimeState:
        return self.arm_states[arm]

    def _clear_arm_target_state(self, arm_state: ArmRuntimeState):
        """清空单臂当前位置控制相关状态。"""
        arm_state.clear_target_state()

    def _capture_current_arm_pose(self, arm: str):
        """抓取当前末端位姿与关节角，作为状态切换的统一输入。"""
        current_position = self._get_current_ee_position(arm)
        current_quat = self._get_current_ee_orientation(arm)
        current_angles = self.robot_interface.get_arm_angles(arm) if self.robot_interface else None
        return current_position, current_quat, current_angles

    def _seed_arm_target_from_current_pose(self, arm: str, arm_state: ArmRuntimeState):
        """
        用当前机械臂姿态重置控制目标。

        这样 grip 激活或 reset 时，新的目标满足：
        q_target = q_current
        p_target = p_current
        可避免第一帧跳变。
        """
        if not self.robot_interface:
            return

        current_position, current_quat, current_angles = self._capture_current_arm_pose(arm)
        arm_state.seed_from_pose(
            position=current_position,
            quat_wxyz=current_quat,
            wrist_roll_deg=current_angles[WRIST_ROLL_INDEX],
            wrist_flex_deg=current_angles[WRIST_FLEX_INDEX],
        )

    def _set_idle_marker_state(self, arm: str, arm_state: ArmRuntimeState, current_position: np.ndarray, current_quat: np.ndarray):
        """退出控制时，让 target marker 与当前 tools_link 严格重合。"""
        arm_state.record_mocap(current_position, current_quat)
        if self.visualizer:
            self.visualizer.update_marker_position(
                f"{arm}_target", current_position, current_quat
            )
            self.visualizer.hide_marker(f"{arm}_goal")
            self.visualizer.hide_frame(f"{arm}_goal_frame")

    def _enter_reference_initialization(self):
        """进入初始化状态前，清空双臂当前位置控制目标。"""
        if self.robot_interface is None:
            return

        for arm_state in self.arm_states.values():
            self._clear_arm_target_state(arm_state)

        ok = self.robot_interface.start_reference_pose_initialization()
        if ok:
            logger.info("📡 X/A long-hold: bridge entered INITIALIZING state")
        else:
            logger.warning("📡 X/A long-hold: failed to enter INITIALIZING state")

    def _activate_position_control(self, arm: str, arm_state: ArmRuntimeState) -> bool:
        """进入 POSITION_CONTROL，并把目标对齐到当前姿态。"""
        bridge_state = getattr(self.robot_interface, "bridge_state", "entering") if self.robot_interface else "entering"
        if bridge_state != "executing":
            logger.warning(
                f"⏸ {arm.upper()} arm position-control request ignored: "
                f"bridge state is {bridge_state}, waiting for EXECUTING"
            )
            return False
        if self.robot_interface and not self.robot_interface.is_motion_gate_ready():
            logger.warning(
                f"⏸ {arm.upper()} arm position-control request ignored: "
                "waiting for /joint_states gate"
            )
            return False

        arm_state.mode = ControlMode.POSITION_CONTROL
        self._seed_arm_target_from_current_pose(arm, arm_state)
        logger.info(f"🔒 {arm.upper()} arm: Position control ACTIVATED (target reset to current position)")
        return True

    def _deactivate_position_control(self, arm: str, arm_state: ArmRuntimeState):
        """退出 POSITION_CONTROL，并保持 marker 稳定。"""
        current_position, current_quat, _ = self._capture_current_arm_pose(arm)
        self._clear_arm_target_state(arm_state)
        self._set_idle_marker_state(arm, arm_state, current_position, current_quat)
        logger.info(f"🔓 {arm.upper()} arm: Position control DEACTIVATED")

    def _apply_position_goal(self, arm: str, arm_state: ArmRuntimeState, goal: ControlGoal):
        """把输入层目标更新到 arm_state，不直接做 IK。"""
        if goal.target_position is not None:
            if goal.metadata and goal.metadata.get("relative_position", False):
                delta = np.asarray(goal.target_position, dtype=float)
                delta_base = self.frame_mapper.map_relative_translation(delta)
                if arm_state.origin_position is not None:
                    arm_state.target_position = arm_state.origin_position + delta_base
                elif self.robot_interface:
                    current_position = self._get_current_ee_position(arm)
                    arm_state.target_position = current_position + delta_base
            else:
                arm_state.target_position = goal.target_position.copy()

            if arm_state.target_position is not None:
                arm_state.goal_position = arm_state.target_position.copy()

        if goal.target_orientation_quat is not None:
            q = np.asarray(goal.target_orientation_quat, dtype=float).reshape(-1)
            if q.size >= 4:
                q_mapped = self._quat_normalize_wxyz(q[:4])
                use_marker_grab_drag = bool(
                    goal.metadata and goal.metadata.get("marker_grab_drag", False)
                )
                if use_marker_grab_drag:
                    q_init = arm_state.origin_target_orientation_quat
                    if q_init is None:
                        q_init = self._get_current_ee_orientation(arm)
                else:
                    q_init = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

                q_out = self.frame_mapper.map_target_orientation(
                    controller_delta_quat_wxyz=q_mapped,
                    origin_target_quat_wxyz=np.asarray(q_init, dtype=float),
                )
                arm_state.target_orientation_quat = self._quat_normalize_wxyz(q_out)

        if goal.wrist_roll_deg is not None:
            if goal.metadata and goal.metadata.get("relative_position", False):
                arm_state.current_wrist_roll = arm_state.origin_wrist_roll_angle + goal.wrist_roll_deg
            else:
                arm_state.current_wrist_roll = goal.wrist_roll_deg

        if goal.wrist_flex_deg is not None:
            if goal.metadata and goal.metadata.get("relative_position", False):
                arm_state.current_wrist_flex = arm_state.origin_wrist_flex_angle + goal.wrist_flex_deg
            else:
                arm_state.current_wrist_flex = goal.wrist_flex_deg

    def _build_ik_request_for_arm(self, arm_name: str, arm_state: ArmRuntimeState):
        """
        为单臂收集 IK 请求。

        返回值为 None 表示当前帧该臂不做 IK，只维持 tools_link 与 target marker 对齐。
        """
        target_name = f"{arm_name}_target"
        goal_name = f"{arm_name}_goal"
        actual_pos = self._get_current_ee_position(arm_name)
        actual_quat = self._get_current_ee_orientation(arm_name)
        motion_gate_ready = self.robot_interface.is_motion_gate_ready() if self.robot_interface else True

        if arm_state.mode != ControlMode.POSITION_CONTROL or not motion_gate_ready:
            self.visualizer.update_marker_position(target_name, actual_pos, actual_quat)
            arm_state.last_mocap_target_position = actual_pos.copy()
            arm_state.last_mocap_target_quaternion = actual_quat.copy()
            self.visualizer.hide_marker(goal_name)
            self.visualizer.hide_frame(f"{target_name}_frame")
            self.visualizer.hide_frame(f"{goal_name}_frame")
            self.visualizer.update_marker_position(f"{arm_name}_actual", actual_pos, actual_quat)
            return None

        if arm_state.target_position is None:
            self.visualizer.update_marker_position(f"{arm_name}_actual", actual_pos, actual_quat)
            return None

        ik_target = arm_state.target_position
        ik_target_quat = arm_state.target_orientation_quat
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
                ik_target_quat = q / n
                arm_state.last_mocap_target_quaternion = ik_target_quat.copy()

        ik_target = actual_pos + IK_TARGET_SMOOTHING * (ik_target - actual_pos)

        if arm_state.target_position is not None:
            self.visualizer.update_marker_position(target_name, arm_state.target_position, arm_state.target_orientation_quat)
            self.visualizer.update_coordinate_frame(f"{target_name}_frame", arm_state.target_position)
        if arm_state.goal_position is not None:
            self.visualizer.update_marker_position(goal_name, arm_state.goal_position)
            self.visualizer.update_coordinate_frame(f"{goal_name}_frame", arm_state.goal_position)

        self.visualizer.update_marker_position(f"{arm_name}_actual", actual_pos, actual_quat)
        return {
            "target": ik_target,
            "target_quat": ik_target_quat,
            "wrist_override": (arm_state.target_orientation_quat is None),
        }

    def _execute_dual_ik_requests(self, ik_requests: Dict[str, Optional[Dict]]):
        """统一执行双臂 IK，并把解算结果回写到 RobotInterface。"""
        if ik_requests["left"] is None and ik_requests["right"] is None:
            return

        left_req = ik_requests["left"]
        right_req = ik_requests["right"]
        left_target = left_req["target"] if left_req is not None else self._get_current_ee_position("left")
        right_target = right_req["target"] if right_req is not None else self._get_current_ee_position("right")
        left_target_quat = left_req["target_quat"] if left_req is not None else None
        right_target_quat = right_req["target_quat"] if right_req is not None else None
        left_before = self.robot_interface.get_arm_angles("left")
        right_before = self.robot_interface.get_arm_angles("right")

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
                self.arm_states["left"].current_wrist_flex,
                self.arm_states["left"].current_wrist_roll,
                left_gripper,
                wrist_override=left_req["wrist_override"],
            )

        if right_req is not None:
            right_gripper = self.robot_interface.get_arm_angles("right")[GRIPPER_INDEX]
            self.robot_interface.update_arm_angles(
                "right",
                right_solution,
                self.arm_states["right"].current_wrist_flex,
                self.arm_states["right"].current_wrist_roll,
                right_gripper,
                wrist_override=right_req["wrist_override"],
            )

        debug_msgs = []
        if left_req is not None:
            left_move = float(np.linalg.norm(np.asarray(left_solution) - np.asarray(left_before)))
            left_err = float(np.linalg.norm(np.asarray(left_target) - self._get_current_ee_position("left")))
            if left_err > 0.02 and left_move < 1e-3:
                debug_msgs.append(
                    f"LEFT target={np.round(left_target, 4)} current={np.round(self._get_current_ee_position('left'), 4)} "
                    f"joint_move={left_move:.6f}"
                )
        if right_req is not None:
            right_move = float(np.linalg.norm(np.asarray(right_solution) - np.asarray(right_before)))
            right_err = float(np.linalg.norm(np.asarray(right_target) - self._get_current_ee_position('right')))
            if right_err > 0.02 and right_move < 1e-3:
                debug_msgs.append(
                    f"RIGHT target={np.round(right_target, 4)} current={np.round(self._get_current_ee_position('right'), 4)} "
                    f"joint_move={right_move:.6f}"
                )
        if debug_msgs:
            logger.warning("Mink solve produced near-zero joint motion despite target error | " + " | ".join(debug_msgs))

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
    
    async def _execute_goal(self, goal: ControlGoal):
        """Execute a control goal."""
        arm_state = self._get_arm_state(goal.arm)

        if goal.metadata and goal.metadata.get("publish_mainpy_joint_target_now", False):
            self._enter_reference_initialization()
            return
        
        # Handle special reset signal from external input timeout.
        if (goal.metadata and goal.metadata.get("reset_target_to_current", False)):
            if self.robot_interface and arm_state.mode == ControlMode.POSITION_CONTROL:
                self._seed_arm_target_from_current_pose(goal.arm, arm_state)
                logger.info(f"🔄 {goal.arm.upper()} arm: Target position reset to current robot position (idle timeout)")
            return
        
        # Handle mode changes (only if mode is specified)
        if goal.mode is not None and goal.mode != arm_state.mode:
            if goal.mode == ControlMode.POSITION_CONTROL:
                if not self._activate_position_control(goal.arm, arm_state):
                    return
                
            elif goal.mode == ControlMode.IDLE:
                self._deactivate_position_control(goal.arm, arm_state)
        
        # Position control uses absolute offset from origin.
        if goal.target_position is not None and arm_state.mode == ControlMode.POSITION_CONTROL:
            self._apply_position_goal(goal.arm, arm_state, goal)
        
        # Handle gripper control (independent of mode)
        if goal.gripper_closed is not None and self.robot_interface:
            self.robot_interface.set_gripper(goal.arm, goal.gripper_closed)
            if self.visualizer:
                self.visualizer.set_gripper_closed(goal.arm, goal.gripper_closed)
    
    def _get_visualizer_joint_angles(self) -> tuple[np.ndarray, np.ndarray]:
        """统一选择可视化驱动角度来源。"""
        if self.config.require_state_feedback:
            left_angles = self.robot_interface.get_actual_arm_angles("left")
            right_angles = self.robot_interface.get_actual_arm_angles("right")
        else:
            left_angles = self.robot_interface.get_arm_angles("left")
            right_angles = self.robot_interface.get_arm_angles("right")
        return left_angles, right_angles

    def _update_visualizer_state(self):
        """统一更新可视化与仿真状态。"""
        if not self.visualizer or not self.robot_interface:
            return

        left_angles, right_angles = self._get_visualizer_joint_angles()
        self.visualizer.update_robot_pose(left_angles, "left")
        self.visualizer.update_robot_pose(right_angles, "right")
        self.visualizer.set_gripper_closed("left", self.robot_interface.get_gripper_closed("left"))
        self.visualizer.set_gripper_closed("right", self.robot_interface.get_gripper_closed("right"))

        self._mujoco_substep_counter += int(self.sim_substeps)
        self.visualizer.step_simulation(substeps=self.sim_substeps)

        if not self.config.require_state_feedback:
            l_sim = self.visualizer.get_joint_angles_deg("left")
            r_sim = self.visualizer.get_joint_angles_deg("right")
            if l_sim is not None:
                self.robot_interface.set_simulated_arm_angles("left", l_sim)
            if r_sim is not None:
                self.robot_interface.set_simulated_arm_angles("right", r_sim)

    def _publish_robot_command(self):
        """统一处理真实机器人命令下发。"""
        if self.config.enable_robot and self.robot_interface:
            self.robot_interface.send_command()
    
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
            for arm in ARM_NAMES:
                if self.arm_states[arm].mode == ControlMode.POSITION_CONTROL:
                    active_arms.append(arm.upper())
            
            if active_arms and self.robot_interface:
                left_angles = self.robot_interface.get_arm_angles("left")
                right_angles = self.robot_interface.get_arm_angles("right")
                details = []
                for arm_name in ARM_NAMES:
                    arm_state = self.arm_states[arm_name]
                    if arm_state.mode != ControlMode.POSITION_CONTROL or arm_state.target_position is None:
                        continue
                    actual_pos = self._get_current_ee_position(arm_name)
                    target_pos = np.asarray(arm_state.target_position, dtype=float).reshape(-1)[:3]
                    pos_err = float(np.linalg.norm(target_pos - actual_pos))
                    details.append(
                        f"{arm_name.upper()}_err={pos_err:.4f}m"
                    )
                bridge_state = getattr(self.robot_interface, "bridge_state", "unknown")
                extra = f" | {' '.join(details)}" if details else ""
                logger.info(
                    f"🤖 Active control: {', '.join(active_arms)} | "
                    f"bridge={bridge_state} | Left: {left_angles.round(1)} | Right: {right_angles.round(1)}{extra}"
                )
    
    @property
    def status(self) -> Dict:
        """Get current control loop status."""
        return {
            "running": self.is_running,
            "left_arm_mode": self.arm_states["left"].mode.value,
            "right_arm_mode": self.arm_states["right"].mode.value,
            "robot_connected": self.robot_interface.is_connected if self.robot_interface else False,
            "left_arm_connected": self.robot_interface.get_arm_connection_status("left") if self.robot_interface else False,
            "right_arm_connected": self.robot_interface.get_arm_connection_status("right") if self.robot_interface else False,
            "visualizer_connected": self.visualizer.is_connected if self.visualizer else False,
        }
