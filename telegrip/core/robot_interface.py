"""
Robot interface module using Mink+MuJoCo for IK and visualization.
"""

import numpy as np
import time
import logging
import mujoco
from typing import Optional, Dict, TYPE_CHECKING, Tuple
from pathlib import Path

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    Node = object  # Dummy base class when ROS2 not available
    JointState = object  # Dummy type
    logging.warning("ROS2 (rclpy) not available - robot control will be disabled")

from ..config import (
    TelegripConfig, NUM_JOINTS, JOINT_NAMES,
    GRIPPER_OPEN_ANGLE, GRIPPER_CLOSED_ANGLE
)
from .mink_kinematics import MinkIKSolver, MinkDualIKSolver, MinkForwardKinematics

logger = logging.getLogger(__name__)


class RobotROSNode(Node):
    """ROS2 node for robot communication."""
    
    def __init__(self, config: TelegripConfig):
        if not ROS2_AVAILABLE:
            raise RuntimeError("ROS2 not available but RobotROSNode was instantiated")
        super().__init__('telegrip_robot_interface')
        self.config = config
        
        self.left_arm_angles = np.zeros(NUM_JOINTS)
        self.right_arm_angles = np.zeros(NUM_JOINTS)
        self.left_sim_angles = np.zeros(NUM_JOINTS)
        self.right_sim_angles = np.zeros(NUM_JOINTS)
        self.left_actual_angles = np.zeros(NUM_JOINTS)
        self.right_actual_angles = np.zeros(NUM_JOINTS)
        
        self.left_cmd_pub = self.create_publisher(JointState, '/left_arm/joint_commands', 10)
        self.right_cmd_pub = self.create_publisher(JointState, '/right_arm/joint_commands', 10)
        
        # Subscribe only to aggregated /joint_states and parse by joint names.
        self.all_state_sub = self.create_subscription(JointState, '/joint_states', self._joint_state_callback, 10)
        
        self.left_arm_connected = False
        self.right_arm_connected = False
        self.last_left_msg_time = 0
        self.last_right_msg_time = 0
        self.connection_timeout = 2.0
        self.require_state_feedback = bool(getattr(config, "require_state_feedback", False))

        # When feedback check is disabled, treat arms as available for command publishing.
        if not self.require_state_feedback:
            self.left_arm_connected = True
            self.right_arm_connected = True
        
        logger.info("ROS2 robot interface node initialized")

    @staticmethod
    def _normalize_joint_state_units(angles: np.ndarray) -> np.ndarray:
        """Convert joint_state values from radians to degrees."""
        arr = np.asarray(angles, dtype=float).reshape(-1)[:NUM_JOINTS]
        if arr.size < NUM_JOINTS:
            return arr
        return np.rad2deg(arr)

    def _extract_arm_angles_from_msg(self, arm: str, msg: JointState) -> Optional[np.ndarray]:
        """Extract one arm's 6 joints from JointState by explicit joint names."""
        if msg is None or not hasattr(msg, "position"):
            return None

        positions = np.asarray(msg.position, dtype=float).reshape(-1)
        if positions.size == 0:
            return None

        names = list(msg.name) if hasattr(msg, "name") and msg.name is not None else []
        if names:
            name_to_idx = {}
            upper = min(len(names), positions.size)
            for i in range(upper):
                n = str(names[i])
                if n not in name_to_idx:
                    name_to_idx[n] = i

            joint_sets = [
                [f"{arm}_joint{i}" for i in range(1, NUM_JOINTS + 1)],
                [f"{arm}/joint{i}" for i in range(1, NUM_JOINTS + 1)],
                [f"joint{i}" for i in range(1, NUM_JOINTS + 1)],
            ]
            for joint_names in joint_sets:
                idxs = []
                ok = True
                for jn in joint_names:
                    idx = name_to_idx.get(jn)
                    if idx is None:
                        # Tolerate namespaced joints, e.g. "robot/right_joint1".
                        for cand, cand_idx in name_to_idx.items():
                            if cand.endswith("/" + jn) or cand.endswith(jn):
                                idx = cand_idx
                                break
                    if idx is None:
                        ok = False
                        break
                    idxs.append(idx)
                if ok:
                    return self._normalize_joint_state_units(positions[idxs])

        # Fallback for legacy per-arm topics without names.
        if positions.size >= NUM_JOINTS:
            return self._normalize_joint_state_units(positions[:NUM_JOINTS])
        return None

    def _update_arm_state_from_msg(self, arm: str, msg: JointState) -> bool:
        """Parse and update one arm state from JointState message."""
        angles = self._extract_arm_angles_from_msg(arm, msg)
        if angles is None or angles.size < NUM_JOINTS:
            return False

        now = time.time()
        if arm == "left":
            self.left_actual_angles = angles.copy()
            self.last_left_msg_time = now
            self.left_arm_connected = True
        else:
            self.right_actual_angles = angles.copy()
            self.last_right_msg_time = now
            self.right_arm_connected = True
        return True
    
    def _joint_state_callback(self, msg: JointState):
        """Handle aggregated /joint_states that contains both arms."""
        self._update_arm_state_from_msg("left", msg)
        self._update_arm_state_from_msg("right", msg)
    
    def publish_command(self, arm: str, angles: np.ndarray):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        # ROS2 sequence<double> is strict: it requires plain Python float values.
        # Also guard against NaN/Inf coming from transient IK/solver states.
        arr = np.asarray(angles, dtype=np.float64).reshape(-1)
        if arr.size < NUM_JOINTS:
            arr = np.pad(arr, (0, NUM_JOINTS - arr.size), mode='constant')
        arr = np.nan_to_num(arr[:NUM_JOINTS], nan=0.0, posinf=0.0, neginf=0.0)
        msg.position = [float(v) for v in arr]
        
        if arm == "left":
            self.left_cmd_pub.publish(msg)
        elif arm == "right":
            self.right_cmd_pub.publish(msg)
    
    def check_connections(self):
        if not self.require_state_feedback:
            self.left_arm_connected = True
            self.right_arm_connected = True
            return

        current_time = time.time()
        
        if self.left_arm_connected:
            if current_time - self.last_left_msg_time > self.connection_timeout:
                self.left_arm_connected = False
                logger.warning("Left arm connection timeout")
        
        if self.right_arm_connected:
            if current_time - self.last_right_msg_time > self.connection_timeout:
                self.right_arm_connected = False
                logger.warning("Right arm connection timeout")


class RobotInterface:
    """Robot interface using Mink+MuJoCo for IK and visualization."""
    
    def __init__(self, config: TelegripConfig):
        self.config = config
        self.require_state_feedback = bool(getattr(config, "require_state_feedback", False))
        self.ros_node = None
        self.is_connected = False
        self.is_engaged = False
        
        self.left_arm_connected = False
        self.right_arm_connected = False
        
        self.left_arm_angles = np.zeros(NUM_JOINTS)
        self.right_arm_angles = np.zeros(NUM_JOINTS)
        self.left_sim_angles = np.zeros(NUM_JOINTS)
        self.right_sim_angles = np.zeros(NUM_JOINTS)
        
        self.joint_limits_min_deg = np.full(NUM_JOINTS, -180.0)
        self.joint_limits_max_deg = np.full(NUM_JOINTS, 180.0)
        self.arm_joint_limits_min_deg = {
            "left": np.full(NUM_JOINTS, -180.0, dtype=float),
            "right": np.full(NUM_JOINTS, -180.0, dtype=float),
        }
        self.arm_joint_limits_max_deg = {
            "left": np.full(NUM_JOINTS, 180.0, dtype=float),
            "right": np.full(NUM_JOINTS, 180.0, dtype=float),
        }
        
        self.fk_solvers = {'left': None, 'right': None}
        self.ik_solvers = {'left': None, 'right': None}
        self.dual_ik_solver = None
        self.arm_sites = {'left': None, 'right': None}
        self.arm_joint_names = {'left': None, 'right': None}
        self.mocap_targets = {'left': None, 'right': None}
        # If True, keep legacy behavior where channel-6 is treated as gripper.
        # For ARM620 scenes, gripper is a separate actuator and this must be False.
        self.gripper_on_joint6 = {'left': True, 'right': True}
        # Default to open gripper on startup.
        self.gripper_closed_state = {'left': False, 'right': False}
        
        self.last_send_time = 0
        
        self.left_arm_errors = 0
        self.right_arm_errors = 0
        self.general_errors = 0
        self.max_arm_errors = 3
        self.max_general_errors = 8
        
        self.initial_left_arm = np.array([90, 30, 90, 90, 90, 0])
        self.initial_right_arm = np.array([-90, -30, -90, -90, -90, 0])
    
    def connect(self) -> bool:
        """Connect to robot via ROS2."""
        if self.is_connected:
            logger.info("Robot interface already connected")
            return True
        
        if not self.config.enable_robot:
            logger.info("Robot interface disabled in config")
            self.is_connected = True
            return True
        
        if not ROS2_AVAILABLE:
            logger.error("ROS2 not available - cannot connect to robot")
            return False
        
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self.ros_node = RobotROSNode(self.config)
            time.sleep(0.5)
            
            self._update_connection_status()

            if self.require_state_feedback:
                self.is_connected = self.left_arm_connected or self.right_arm_connected
            else:
                self.left_arm_connected = True
                self.right_arm_connected = True
                self.is_connected = True
            
            if self.is_connected:
                logger.info(f"🤖 Robot interface connected via ROS2: Left={self.left_arm_connected}, Right={self.right_arm_connected}")
            else:
                logger.warning("⚠️ ROS2 node created but no arms detected yet")
                self.is_connected = True

            if not self.require_state_feedback:
                logger.info("✅ Feedback check disabled: publish commands directly to control topics")
            
            return True
            
        except Exception as e:
            logger.error(f"❌ Robot ROS2 connection failed: {e}")
            self.is_connected = False
            return False
    
    def _update_connection_status(self):
        if self.ros_node:
            if not self.require_state_feedback:
                self.left_arm_connected = True
                self.right_arm_connected = True
                return

            rclpy.spin_once(self.ros_node, timeout_sec=0.01)
            self.ros_node.check_connections()
            self.left_arm_connected = self.ros_node.left_arm_connected
            self.right_arm_connected = self.ros_node.right_arm_connected
            
            if self.left_arm_connected:
                self.left_arm_angles = self.ros_node.left_arm_angles.copy()
            if self.right_arm_connected:
                self.right_arm_angles = self.ros_node.right_arm_angles.copy()

    def wait_for_joint_state_snapshot(self, timeout_s: float = 1.0) -> Dict[str, np.ndarray]:
        """Spin ROS subscriptions briefly and return latest raw joint_state samples.

        This is used at startup to initialize MuJoCo pose from real robot state
        exactly once, even when continuous feedback mode is disabled.
        """
        snapshot: Dict[str, np.ndarray] = {}
        if not (ROS2_AVAILABLE and self.ros_node and rclpy.ok()):
            return snapshot

        timeout_s = max(0.0, float(timeout_s))
        deadline = time.perf_counter() + timeout_s

        while True:
            try:
                rclpy.spin_once(self.ros_node, timeout_sec=0.02)
            except Exception as e:
                logger.debug(f"startup joint_state spin_once failed: {e}")
                break

            if self.ros_node.last_left_msg_time > 0.0:
                snapshot["left"] = self.ros_node.left_actual_angles.copy()
            if self.ros_node.last_right_msg_time > 0.0:
                snapshot["right"] = self.ros_node.right_actual_angles.copy()

            if "left" in snapshot and "right" in snapshot:
                break
            if time.perf_counter() >= deadline:
                break

        return snapshot
    
    def setup_mink_kinematics(self, mujoco_scene_path: str, end_effector_site: str = "tools_link",
                             joint_names: list = None):
        """
        Setup Mink-based IK and FK solvers.
        
        Args:
            mujoco_scene_path: Path to MuJoCo scene XML file
            end_effector_site: Name of end effector site
            joint_names: List of joint names
        """
        try:
            scene_path = Path(mujoco_scene_path)
            if not scene_path.exists():
                logger.error(f"MuJoCo scene file not found: {mujoco_scene_path}")
                return False
            
            model = mujoco.MjModel.from_xml_path(str(scene_path))
            
            site_names = []
            for sid in range(model.nsite):
                sname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, sid)
                if sname:
                    site_names.append(sname)

            suffix = f"/{end_effector_site}"
            site_candidates = [n for n in site_names if n == end_effector_site or n.endswith(suffix)]

            def _resolve_arm_site(arm: str) -> str:
                preferred = f"{arm}/{end_effector_site}"
                if preferred in site_names:
                    return preferred
                arm_pref = [n for n in site_candidates if n.startswith(f"{arm}/")]
                if arm_pref:
                    return arm_pref[0]
                if end_effector_site in site_names:
                    return end_effector_site
                return site_candidates[0] if site_candidates else end_effector_site

            arm_site = {
                "left": _resolve_arm_site("left"),
                "right": _resolve_arm_site("right"),
            }
            self.arm_sites = arm_site.copy()

            dual_namespaced = arm_site["left"] != arm_site["right"]
            self.mocap_targets = {
                "left": "left_target" if dual_namespaced else "target",
                "right": "right_target" if dual_namespaced else "target",
            }
            if dual_namespaced:
                arm_joint_names = {
                    "left": [f"left/joint{i}" for i in range(1, NUM_JOINTS + 1)],
                    "right": [f"right/joint{i}" for i in range(1, NUM_JOINTS + 1)],
                }
            else:
                if joint_names is None:
                    arm_joint_names = {
                        "left": [f"joint{i}" for i in range(1, NUM_JOINTS + 1)],
                        "right": [f"joint{i}" for i in range(1, NUM_JOINTS + 1)],
                    }
                else:
                    arm_joint_names = {
                        "left": joint_names[:NUM_JOINTS],
                        "right": joint_names[:NUM_JOINTS],
                    }

            self.arm_joint_names = {
                "left": list(arm_joint_names["left"][:NUM_JOINTS]),
                "right": list(arm_joint_names["right"][:NUM_JOINTS]),
            }

            arm_limits_min_deg = {
                "left": np.full(NUM_JOINTS, -180.0, dtype=float),
                "right": np.full(NUM_JOINTS, -180.0, dtype=float),
            }
            arm_limits_max_deg = {
                "left": np.full(NUM_JOINTS, 180.0, dtype=float),
                "right": np.full(NUM_JOINTS, 180.0, dtype=float),
            }

            for arm in ('left', 'right'):
                for i, jname in enumerate(self.arm_joint_names[arm]):
                    try:
                        jid = model.joint(jname).id
                        if model.jnt_limited[jid]:
                            lower, upper = model.jnt_range[jid]
                            arm_limits_min_deg[arm][i] = np.rad2deg(lower)
                            arm_limits_max_deg[arm][i] = np.rad2deg(upper)
                    except Exception:
                        continue

            self.arm_joint_limits_min_deg = {
                "left": arm_limits_min_deg["left"].copy(),
                "right": arm_limits_min_deg["right"].copy(),
            }
            self.arm_joint_limits_max_deg = {
                "left": arm_limits_max_deg["left"].copy(),
                "right": arm_limits_max_deg["right"].copy(),
            }

            # 兼容旧代码：保留左臂默认限位快照。
            self.joint_limits_min_deg = self.arm_joint_limits_min_deg["left"].copy()
            self.joint_limits_max_deg = self.arm_joint_limits_max_deg["left"].copy()

            # Detect whether this scene has a separate gripper actuator.
            for arm in ['left', 'right']:
                self.gripper_on_joint6[arm] = True
                for act_name in (
                    f"{arm}/robotiq_2f85_v4_actuator",
                    "robotiq_2f85_v4_actuator",
                ):
                    try:
                        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
                        if aid >= 0:
                            self.gripper_on_joint6[arm] = False
                            break
                    except Exception:
                        continue

            self.dual_ik_solver = None
            self.ik_solvers = {'left': None, 'right': None}

            if dual_namespaced:
                # 双臂联合模式：一个Mink配置内同时优化左右臂。
                self.dual_ik_solver = MinkDualIKSolver(
                    xml_path=str(scene_path),
                    left_end_effector_site=arm_site["left"],
                    right_end_effector_site=arm_site["right"],
                    left_joint_names=self.arm_joint_names["left"],
                    right_joint_names=self.arm_joint_names["right"],
                    left_joint_limits_min_deg=arm_limits_min_deg["left"],
                    left_joint_limits_max_deg=arm_limits_max_deg["left"],
                    right_joint_limits_min_deg=arm_limits_min_deg["right"],
                    right_joint_limits_max_deg=arm_limits_max_deg["right"],
                    position_cost=self.config.mink_position_cost,
                    orientation_cost=self.config.mink_orientation_cost,
                    posture_cost=self.config.mink_posture_cost,
                    lm_damping=self.config.mink_lm_damping,
                    solve_damping=self.config.mink_solve_damping,
                    dt=self.config.mink_dt,
                    max_iters=self.config.mink_max_iters,
                    pos_threshold=self.config.mink_position_error_threshold,
                    ori_threshold=self.config.mink_orientation_error_threshold,
                )
            else:
                for arm in ['left', 'right']:
                    jnames = self.arm_joint_names[arm]
                    self.ik_solvers[arm] = MinkIKSolver(
                        xml_path=str(scene_path),
                        end_effector_site=arm_site[arm],
                        joint_names=jnames,
                        joint_limits_min_deg=self.joint_limits_min_deg,
                        joint_limits_max_deg=self.joint_limits_max_deg,
                        num_ik_joints=NUM_JOINTS,
                        position_cost=self.config.mink_position_cost,
                        orientation_cost=self.config.mink_orientation_cost,
                        posture_cost=self.config.mink_posture_cost,
                        lm_damping=self.config.mink_lm_damping,
                        solve_damping=self.config.mink_solve_damping,
                        dt=self.config.mink_dt,
                        max_iters=self.config.mink_max_iters,
                        pos_threshold=self.config.mink_position_error_threshold,
                        ori_threshold=self.config.mink_orientation_error_threshold,
                    )

            for arm in ['left', 'right']:
                jnames = self.arm_joint_names[arm]
                self.fk_solvers[arm] = MinkForwardKinematics(
                    xml_path=str(scene_path),
                    end_effector_site=arm_site[arm],
                    num_joints=NUM_JOINTS,
                    joint_names=jnames,
                )

            logger.info(f"✓ Mink kinematics solvers initialized for both arms")
            logger.info(f"  Joint IK mode: {'shared dual solver' if self.dual_ik_solver is not None else 'per-arm solvers'}")
            logger.info(f"  Scene: {scene_path.name}")
            logger.info(f"  End effectors: left={arm_site['left']} right={arm_site['right']}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to setup Mink kinematics: {e}")
            return False
    
    def get_current_end_effector_position(self, arm: str) -> np.ndarray:
        """Get current end effector position."""
        angles = self.left_arm_angles if arm == "left" else self.right_arm_angles
        
        if self.fk_solvers[arm]:
            position, _ = self.fk_solvers[arm].compute(angles)
            return position
        else:
            return np.array([0.3, 0.0, 0.4])
    
    def solve_ik(self, arm: str, target_position: np.ndarray, 
                 target_orientation: Optional[np.ndarray] = None) -> np.ndarray:
        """Solve inverse kinematics using Mink."""
        # Match arm_arm620_dual pattern: seed IK with current measured/simulated state.
        current_angles = self.get_actual_arm_angles(arm)

        if self.dual_ik_solver is not None:
            other_arm = "right" if arm == "left" else "left"
            other_pos = self.get_current_end_effector_position(other_arm)
            other_ori = None
            l_pos = target_position if arm == "left" else other_pos
            r_pos = target_position if arm == "right" else other_pos
            l_ori = target_orientation if arm == "left" else other_ori
            r_ori = target_orientation if arm == "right" else other_ori
            l_sol, r_sol = self.dual_ik_solver.solve(
                left_target_position=l_pos,
                right_target_position=r_pos,
                left_target_orientation_quat=l_ori,
                right_target_orientation_quat=r_ori,
                current_left_angles_deg=self.get_actual_arm_angles("left"),
                current_right_angles_deg=self.get_actual_arm_angles("right"),
                left_mocap_name=self.mocap_targets.get("left"),
                right_mocap_name=self.mocap_targets.get("right"),
            )
            return l_sol if arm == "left" else r_sol

        if self.ik_solvers[arm]:
            return self.ik_solvers[arm].solve(
                target_position,
                target_orientation,
                current_angles,
                mocap_name=self.mocap_targets.get(arm),
            )
        else:
            return current_angles[:NUM_JOINTS]

    def solve_dual_ik(
        self,
        left_target_position: np.ndarray,
        right_target_position: np.ndarray,
        left_target_orientation: Optional[np.ndarray] = None,
        right_target_orientation: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Solve dual-arm IK in one shared Mink optimization step."""
        if self.dual_ik_solver is not None:
            return self.dual_ik_solver.solve(
                left_target_position=left_target_position,
                right_target_position=right_target_position,
                left_target_orientation_quat=left_target_orientation,
                right_target_orientation_quat=right_target_orientation,
                current_left_angles_deg=self.get_actual_arm_angles("left"),
                current_right_angles_deg=self.get_actual_arm_angles("right"),
                left_mocap_name=self.mocap_targets.get("left"),
                right_mocap_name=self.mocap_targets.get("right"),
            )

        left_solution = self.solve_ik("left", left_target_position, left_target_orientation)
        right_solution = self.solve_ik("right", right_target_position, right_target_orientation)
        return left_solution, right_solution
    
    def clamp_joint_angles(self, joint_angles: np.ndarray, arm: str = "left") -> np.ndarray:
        """Clamp joint angles to safe limits for the given arm."""
        processed_angles = joint_angles.copy()

        limits_min = self.arm_joint_limits_min_deg.get(arm, self.joint_limits_min_deg)
        limits_max = self.arm_joint_limits_max_deg.get(arm, self.joint_limits_max_deg)

        shoulder_pan_idx = 0
        shoulder_pan_angle = processed_angles[shoulder_pan_idx]
        min_limit = limits_min[shoulder_pan_idx]
        max_limit = limits_max[shoulder_pan_idx]

        if shoulder_pan_angle < min_limit or shoulder_pan_angle > max_limit:
            for offset in [-360.0, 360.0]:
                wrapped_angle = shoulder_pan_angle + offset
                if min_limit <= wrapped_angle <= max_limit:
                    processed_angles[shoulder_pan_idx] = wrapped_angle
                    break

        return np.clip(processed_angles, limits_min, limits_max)
    
    def update_arm_angles(
        self,
        arm: str,
        ik_angles: np.ndarray,
        wrist_flex: float,
        wrist_roll: float,
        gripper: float,
        wrist_override: bool = False,
    ):
        """Update joint angles with IK solution.

        When `wrist_override` is True, wrist joints are overridden by direct
        controller commands. Otherwise, full IK output is preserved to improve
        end-effector target tracking.
        """
        target_angles = self.left_arm_angles if arm == "left" else self.right_arm_angles

        ik = np.asarray(ik_angles, dtype=float).reshape(-1)
        n = min(NUM_JOINTS, ik.size)
        target_angles[:n] = ik[:n]

        if wrist_override and NUM_JOINTS >= 5:
            target_angles[3] = wrist_flex
            target_angles[4] = wrist_roll

        # Legacy path: some robots map gripper to channel-6 directly.
        # ARM620 should NOT do this (joint6 is wrist yaw; gripper is separate actuator).
        if self.gripper_on_joint6.get(arm, True) and NUM_JOINTS >= 6:
            target_angles[5] = np.clip(gripper, GRIPPER_OPEN_ANGLE, GRIPPER_CLOSED_ANGLE)
        
        clamped_angles = self.clamp_joint_angles(target_angles, arm=arm)
        if self.gripper_on_joint6.get(arm, True) and NUM_JOINTS >= 6:
            clamped_angles[5] = target_angles[5]
        
        if arm == "left":
            self.left_arm_angles = clamped_angles
        else:
            self.right_arm_angles = clamped_angles
    
    def engage(self) -> bool:
        """Engage robot motors."""
        if not self.is_connected:
            logger.warning("Cannot engage robot: not connected")
            return False
        
        self.is_engaged = True
        logger.info("🔌 Robot motors ENGAGED")
        return True
    
    def disengage(self) -> bool:
        """Disengage robot motors."""
        if not self.is_connected:
            logger.info("Robot already disconnected")
            return True
        
        try:
            self.return_to_initial_position()
            self.is_engaged = False
            logger.info("🔌 Robot motors DISENGAGED")
            return True
        except Exception as e:
            logger.error(f"Error disengaging robot: {e}")
            return False
    
    def send_command(self) -> bool:
        """Send joint angles to robot via ROS2."""
        if not self.is_connected or not self.is_engaged:
            return False
        
        current_time = time.time()
        if current_time - self.last_send_time < self.config.send_interval:
            return True
        
        try:
            success = True
            if self.require_state_feedback:
                self._update_connection_status()
            
            if self.ros_node and (self.left_arm_connected or not self.require_state_feedback):
                try:
                    self.ros_node.publish_command("left", self.left_arm_angles)
                    self.ros_node.left_arm_angles = self.left_arm_angles.copy()
                except Exception as e:
                    logger.error(f"Error sending left arm command: {e}")
                    self.left_arm_errors += 1
                    if self.left_arm_errors > self.max_arm_errors:
                        self.left_arm_connected = False
                    success = False
            
            if self.ros_node and (self.right_arm_connected or not self.require_state_feedback):
                try:
                    self.ros_node.publish_command("right", self.right_arm_angles)
                    self.ros_node.right_arm_angles = self.right_arm_angles.copy()
                except Exception as e:
                    logger.error(f"Error sending right arm command: {e}")
                    self.right_arm_errors += 1
                    if self.right_arm_errors > self.max_arm_errors:
                        self.right_arm_connected = False
                    success = False
            
            self.last_send_time = current_time
            return success
            
        except Exception as e:
            logger.error(f"Error sending robot command: {e}")
            self.general_errors += 1
            if self.general_errors > self.max_general_errors:
                self.is_connected = False
            return False
    
    def set_gripper(self, arm: str, closed: bool):
        """Set gripper state."""
        angle = GRIPPER_CLOSED_ANGLE if closed else GRIPPER_OPEN_ANGLE
        self.gripper_closed_state[arm] = bool(closed)

        # For models with separate gripper actuator (e.g., ARM620+Robotiq),
        # do not remap gripper into joint6.
        if not self.gripper_on_joint6.get(arm, True):
            return
        
        if arm == "left":
            self.left_arm_angles[5] = angle
        elif arm == "right":
            self.right_arm_angles[5] = angle

    def get_gripper_closed(self, arm: str) -> bool:
        """Get desired gripper state for one arm."""
        return bool(self.gripper_closed_state.get(arm, False))
    
    def get_arm_angles(self, arm: str) -> np.ndarray:
        """Get current joint angles."""
        return (self.left_arm_angles if arm == "left" else self.right_arm_angles).copy()
    
    def get_arm_angles_for_visualization(self, arm: str) -> np.ndarray:
        """Get joint angles for visualization."""
        return self.get_arm_angles(arm)
    
    def get_actual_arm_angles(self, arm: str) -> np.ndarray:
        """Get actual joint angles from robot hardware."""
        # In no-feedback mode, there is no reliable joint state subscriber data.
        # Prefer MuJoCo simulated feedback if available.
        if not self.require_state_feedback:
            sim = getattr(self, "left_sim_angles", np.zeros(NUM_JOINTS)) if arm == "left" else getattr(self, "right_sim_angles", np.zeros(NUM_JOINTS))
            if np.any(np.abs(sim) > 1e-9):
                return sim.copy()
            return self.get_arm_angles(arm)

        try:
            if self.ros_node:
                self._update_connection_status()
                
                if arm == "left" and self.left_arm_connected:
                    return self.ros_node.left_actual_angles.copy()
                elif arm == "right" and self.right_arm_connected:
                    return self.ros_node.right_actual_angles.copy()
        except Exception as e:
            logger.debug(f"Could not get actual angles: {e}")
        
        return self.get_arm_angles(arm)

    def set_simulated_arm_angles(self, arm: str, angles_deg: np.ndarray):
        """Update simulated joint-angle feedback from MuJoCo state."""
        arr = np.asarray(angles_deg[:NUM_JOINTS], dtype=float)
        if arm == "left":
            self.left_sim_angles = arr.copy()
        else:
            self.right_sim_angles = arr.copy()
    
    def return_to_initial_position(self):
        """Return arms to initial position."""
        logger.info("Returning to initial position...")
        self.left_arm_angles = self.initial_left_arm.copy()
        self.right_arm_angles = self.initial_right_arm.copy()
        
        for _ in range(5):
            self.send_command()
            time.sleep(0.1)
    
    def disconnect(self):
        """Disconnect from robot."""
        if not self.is_connected:
            return
        
        try:
            if self.is_engaged:
                self.disengage()
            
            if self.ros_node:
                self.ros_node.destroy_node()
                self.ros_node = None
            
            if ROS2_AVAILABLE and rclpy.ok():
                rclpy.shutdown()
            
            self.is_connected = False
            logger.info("Robot interface disconnected")
            
        except Exception as e:
            logger.error(f"Error disconnecting robot: {e}")

    def get_arm_connection_status(self, arm: str) -> bool:
        """Return per-arm connection status used by status API/UI."""
        if not self.require_state_feedback and self.is_connected:
            return True
        return self.left_arm_connected if arm == "left" else self.right_arm_connected
