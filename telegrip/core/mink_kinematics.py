"""
MuJoCo-based kinematics using mink library for IK solving.
Replaces PyBullet-based IK solver with mink's differential IK.
"""

import numpy as np
import mujoco
from typing import Optional, Tuple, List
import logging
import time
from pathlib import Path

import mink

logger = logging.getLogger(__name__)


class MinkIKSolver:
    """Inverse kinematics solver using mink library with MuJoCo."""
    
    def __init__(self, xml_path: str, end_effector_site: str = "attachment_site",
                 joint_names: list = None, joint_limits_min_deg: np.ndarray = None,
                 joint_limits_max_deg: np.ndarray = None, num_ik_joints: int = 6,
                 position_cost: float = 1.0, orientation_cost: float = 1.0,
                 posture_cost: float = 1e-2, lm_damping: float = 1.0,
                 solve_damping: float = 1e-3, dt: float = 0.005,
                 max_iters: int = 20, pos_threshold: float = 1e-4,
                 ori_threshold: float = 1e-3):
        """
        Initialize mink IK solver.
        
        Args:
            xml_path: Path to MuJoCo XML model file
            end_effector_site: Name of the end effector site in the model
            joint_names: List of joint names to control
            joint_limits_min_deg: Joint limits minimum (degrees)
            joint_limits_max_deg: Joint limits maximum (degrees)
            num_ik_joints: Number of joints to use for IK (default 6)
        """
        self.xml_path = Path(xml_path)
        if not self.xml_path.exists():
            raise FileNotFoundError(f"MuJoCo model file not found: {xml_path}")
        
        self.model = mujoco.MjModel.from_xml_path(str(self.xml_path))
        self.data = mujoco.MjData(self.model)
        
        self.end_effector_site = end_effector_site
        self.num_ik_joints = num_ik_joints
        
        if joint_names is None:
            joint_names = [f"joint{i+1}" for i in range(num_ik_joints)]
        self.joint_names = list(joint_names[:num_ik_joints])

        self.joint_qpos_indices = np.array(
            [self.model.joint(name).qposadr[0] for name in self.joint_names], dtype=int
        )
        
        if joint_limits_min_deg is None:
            joint_limits_min_deg = np.full(num_ik_joints, -180.0)
        if joint_limits_max_deg is None:
            joint_limits_max_deg = np.full(num_ik_joints, 180.0)
            
        self.joint_limits_min_deg = joint_limits_min_deg
        self.joint_limits_max_deg = joint_limits_max_deg
        
        self.configuration = mink.Configuration(self.model)
        
        self.end_effector_task = mink.FrameTask(
            frame_name=end_effector_site,
            frame_type="site",
            position_cost=float(position_cost),
            orientation_cost=float(orientation_cost),
            lm_damping=float(lm_damping),
        )
        
        self.posture_task = mink.PostureTask(model=self.model, cost=float(posture_cost))
        
        self.tasks: List = [self.end_effector_task, self.posture_task]
        self._mocap_ids = {}

        self.solver = "daqp"
        self.dt = float(dt)
        self.max_iters = int(max_iters)
        self.pos_threshold = float(pos_threshold)
        self.ori_threshold = float(ori_threshold)
        self.solve_damping = float(solve_damping)
        
        logger.info(f"MinkIKSolver initialized with model: {self.xml_path.name}")
    
    def solve(self, target_position: np.ndarray, 
              target_orientation_quat: Optional[np.ndarray] = None,
              current_angles_deg: np.ndarray = None,
              mocap_name: Optional[str] = None) -> np.ndarray:
        """
        Solve inverse kinematics for target position.
        
        Args:
            target_position: Target end effector position [x, y, z] in meters
            target_orientation_quat: Target orientation quaternion (optional)
            current_angles_deg: Current joint angles in degrees
            
        Returns:
            Joint angles in degrees for controlled joints
        """
        if current_angles_deg is None:
            current_angles_deg = np.zeros(self.num_ik_joints)
        
        current_angles_rad = np.deg2rad(current_angles_deg[:self.num_ik_joints])

        # Preserve non-controlled joints and only overwrite configured arm joints.
        full_qpos = self.configuration.q.copy()
        full_qpos[self.joint_qpos_indices] = current_angles_rad
        self.configuration.update(full_qpos)
        
        self.posture_task.set_target_from_configuration(self.configuration)
        
        T_wt = None
        if mocap_name is not None:
            try:
                mocap_id = self._mocap_ids.get(mocap_name)
                if mocap_id is None:
                    body = self.model.body(mocap_name)
                    mocap_id = int(body.mocapid[0])
                    if mocap_id < 0:
                        raise ValueError(f"Body '{mocap_name}' is not mocap")
                    self._mocap_ids[mocap_name] = mocap_id

                if target_position is not None:
                    self.data.mocap_pos[mocap_id] = np.asarray(target_position, dtype=float)
                if target_orientation_quat is not None:
                    q = np.asarray(target_orientation_quat, dtype=float).reshape(-1)
                    if q.size >= 4:
                        # MuJoCo mocap quaternion uses [w, x, y, z]
                        self.data.mocap_quat[mocap_id] = q[:4]

                T_wt = mink.SE3.from_mocap_name(self.model, self.data, mocap_name)
            except Exception:
                T_wt = None

        if T_wt is None:
            T_wt = mink.SE3.from_translation(target_position)
        self.end_effector_task.set_target(T_wt)
        
        for _ in range(self.max_iters):
            vel = mink.solve_ik(
                self.configuration, 
                self.tasks, 
                self.dt, 
                self.solver, 
                damping=self.solve_damping
            )
            self.configuration.integrate_inplace(vel, self.dt)
            
            err = self.end_effector_task.compute_error(self.configuration)
            pos_achieved = np.linalg.norm(err[:3]) <= self.pos_threshold

            # If an orientation target is provided, enforce orientation convergence too.
            if target_orientation_quat is not None:
                ori_achieved = np.linalg.norm(err[3:]) <= self.ori_threshold
                if pos_achieved and ori_achieved:
                    break
            elif pos_achieved:
                break
        
        solution_rad = self.configuration.q[self.joint_qpos_indices]
        solution_deg = np.rad2deg(solution_rad)
        
        solution_deg = np.clip(
            solution_deg,
            self.joint_limits_min_deg[:self.num_ik_joints],
            self.joint_limits_max_deg[:self.num_ik_joints]
        )
        
        return solution_deg
    
    def compute_fk(self, joint_angles_deg: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute forward kinematics.
        
        Args:
            joint_angles_deg: Joint angles in degrees
            
        Returns:
            Tuple of (position, quaternion)
        """
        joint_angles_rad = np.deg2rad(joint_angles_deg[:self.num_ik_joints])
        
        full_qpos = np.zeros(self.model.nq)
        full_qpos[:self.num_ik_joints] = joint_angles_rad
        
        self.data.qpos[:] = full_qpos
        mujoco.mj_forward(self.model, self.data)
        
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, self.end_effector_site)
        position = self.data.site_xpos[site_id].copy()
        
        rotation_matrix = self.data.site_xmat[site_id].reshape(3, 3)
        quaternion = self._mat_to_quat(rotation_matrix)
        
        return position, quaternion
    
    @staticmethod
    def _mat_to_quat(mat: np.ndarray) -> np.ndarray:
        """Convert rotation matrix to quaternion [w, x, y, z]."""
        trace = np.trace(mat)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (mat[2, 1] - mat[1, 2]) * s
            y = (mat[0, 2] - mat[2, 0]) * s
            z = (mat[1, 0] - mat[0, 1]) * s
        elif mat[0, 0] > mat[1, 1] and mat[0, 0] > mat[2, 2]:
            s = 2.0 * np.sqrt(1.0 + mat[0, 0] - mat[1, 1] - mat[2, 2])
            w = (mat[2, 1] - mat[1, 2]) / s
            x = 0.25 * s
            y = (mat[0, 1] + mat[1, 0]) / s
            z = (mat[0, 2] + mat[2, 0]) / s
        elif mat[1, 1] > mat[2, 2]:
            s = 2.0 * np.sqrt(1.0 + mat[1, 1] - mat[0, 0] - mat[2, 2])
            w = (mat[0, 2] - mat[2, 0]) / s
            x = (mat[0, 1] + mat[1, 0]) / s
            y = 0.25 * s
            z = (mat[1, 2] + mat[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + mat[2, 2] - mat[0, 0] - mat[1, 1])
            w = (mat[1, 0] - mat[0, 1]) / s
            x = (mat[0, 2] + mat[2, 0]) / s
            y = (mat[1, 2] + mat[2, 1]) / s
            z = 0.25 * s
        return np.array([w, x, y, z])


class MinkDualIKSolver:
    """Single Mink solver that optimizes both arms jointly."""

    def __init__(self, xml_path: str, left_end_effector_site: str, right_end_effector_site: str,
                 left_joint_names: list, right_joint_names: list,
                 left_joint_limits_min_deg: np.ndarray, left_joint_limits_max_deg: np.ndarray,
                 right_joint_limits_min_deg: np.ndarray, right_joint_limits_max_deg: np.ndarray,
                 position_cost: float = 1.0, orientation_cost: float = 1.0,
                 posture_cost: float = 1e-2, lm_damping: float = 1.0,
                 solve_damping: float = 1e-3, dt: float = 0.005,
                 max_iters: int = 20, pos_threshold: float = 1e-4, ori_threshold: float = 1e-3):
        self.xml_path = Path(xml_path)
        if not self.xml_path.exists():
            raise FileNotFoundError(f"MuJoCo model file not found: {xml_path}")

        self.model = mujoco.MjModel.from_xml_path(str(self.xml_path))
        self.data = mujoco.MjData(self.model)
        self.configuration = mink.Configuration(self.model)

        self.left_joint_names = list(left_joint_names)
        self.right_joint_names = list(right_joint_names)
        self.left_joint_qpos_indices = np.array(
            [self.model.joint(name).qposadr[0] for name in self.left_joint_names], dtype=int
        )
        self.right_joint_qpos_indices = np.array(
            [self.model.joint(name).qposadr[0] for name in self.right_joint_names], dtype=int
        )

        self.left_joint_limits_min_deg = np.asarray(left_joint_limits_min_deg, dtype=float)
        self.left_joint_limits_max_deg = np.asarray(left_joint_limits_max_deg, dtype=float)
        self.right_joint_limits_min_deg = np.asarray(right_joint_limits_min_deg, dtype=float)
        self.right_joint_limits_max_deg = np.asarray(right_joint_limits_max_deg, dtype=float)

        self.left_ee_task = mink.FrameTask(
            frame_name=left_end_effector_site,
            frame_type="site",
            position_cost=float(position_cost),
            orientation_cost=float(orientation_cost),
            lm_damping=float(lm_damping),
        )
        self.right_ee_task = mink.FrameTask(
            frame_name=right_end_effector_site,
            frame_type="site",
            position_cost=float(position_cost),
            orientation_cost=float(orientation_cost),
            lm_damping=float(lm_damping),
        )
        self.posture_task = mink.PostureTask(model=self.model, cost=float(posture_cost))
        self.tasks: List = [self.left_ee_task, self.right_ee_task, self.posture_task]

        self.solver = "daqp"
        self.dt = float(dt)
        self.max_iters = int(max_iters)
        self.pos_threshold = float(pos_threshold)
        self.ori_threshold = float(ori_threshold)
        self.solve_damping = float(solve_damping)

        self._mocap_ids = {}
        # IK调用频率统计（1秒窗口平均）
        self._ik_window_start_t = time.perf_counter()
        self._ik_window_calls = 0
        self._ik_window_elapsed_sum_ms = 0.0

    def _target_se3(self, target_position: np.ndarray, target_orientation_quat: Optional[np.ndarray],
                    mocap_name: Optional[str]) -> mink.SE3:
        T_wt = None
        if mocap_name is not None:
            try:
                mocap_id = self._mocap_ids.get(mocap_name)
                if mocap_id is None:
                    body = self.model.body(mocap_name)
                    mocap_id = int(body.mocapid[0])
                    if mocap_id < 0:
                        raise ValueError(f"Body '{mocap_name}' is not mocap")
                    self._mocap_ids[mocap_name] = mocap_id

                if target_position is not None:
                    self.data.mocap_pos[mocap_id] = np.asarray(target_position, dtype=float)
                if target_orientation_quat is not None:
                    q = np.asarray(target_orientation_quat, dtype=float).reshape(-1)
                    if q.size >= 4:
                        # MuJoCo mocap quaternion uses [w, x, y, z]
                        self.data.mocap_quat[mocap_id] = q[:4]

                T_wt = mink.SE3.from_mocap_name(self.model, self.data, mocap_name)
            except Exception:
                T_wt = None

        if T_wt is not None:
            return T_wt

        if target_orientation_quat is not None:
            q = np.asarray(target_orientation_quat, dtype=float).reshape(-1)[:4]
            if q.size == 4:
                return mink.SE3.from_rotation_and_translation(
                    rotation=mink.SO3(wxyz=q),
                    translation=np.asarray(target_position, dtype=float),
                )

        return mink.SE3.from_translation(np.asarray(target_position, dtype=float))

    def solve(self,
              left_target_position: np.ndarray, right_target_position: np.ndarray,
              left_target_orientation_quat: Optional[np.ndarray] = None,
              right_target_orientation_quat: Optional[np.ndarray] = None,
              current_left_angles_deg: Optional[np.ndarray] = None,
              current_right_angles_deg: Optional[np.ndarray] = None,
              left_mocap_name: Optional[str] = None,
              right_mocap_name: Optional[str] = None) -> Tuple[np.ndarray, np.ndarray]:
        call_start_t = time.perf_counter()
        if current_left_angles_deg is None:
            current_left_angles_deg = np.zeros(len(self.left_joint_qpos_indices))
        if current_right_angles_deg is None:
            current_right_angles_deg = np.zeros(len(self.right_joint_qpos_indices))

        full_qpos = self.configuration.q.copy()
        full_qpos[self.left_joint_qpos_indices] = np.deg2rad(np.asarray(current_left_angles_deg, dtype=float)[:len(self.left_joint_qpos_indices)])
        full_qpos[self.right_joint_qpos_indices] = np.deg2rad(np.asarray(current_right_angles_deg, dtype=float)[:len(self.right_joint_qpos_indices)])
        self.configuration.update(full_qpos)

        self.posture_task.set_target_from_configuration(self.configuration)

        self.left_ee_task.set_target(
            self._target_se3(left_target_position, left_target_orientation_quat, left_mocap_name)
        )
        self.right_ee_task.set_target(
            self._target_se3(right_target_position, right_target_orientation_quat, right_mocap_name)
        )

        for _ in range(self.max_iters):
            vel = mink.solve_ik(
                self.configuration,
                self.tasks,
                self.dt,
                self.solver,
                damping=self.solve_damping,
            )
            self.configuration.integrate_inplace(vel, self.dt)

            left_err = self.left_ee_task.compute_error(self.configuration)
            right_err = self.right_ee_task.compute_error(self.configuration)

            left_pos_ok = np.linalg.norm(left_err[:3]) <= self.pos_threshold
            right_pos_ok = np.linalg.norm(right_err[:3]) <= self.pos_threshold

            left_ori_ok = True
            right_ori_ok = True
            if left_target_orientation_quat is not None:
                left_ori_ok = np.linalg.norm(left_err[3:]) <= self.ori_threshold
            if right_target_orientation_quat is not None:
                right_ori_ok = np.linalg.norm(right_err[3:]) <= self.ori_threshold

            if left_pos_ok and right_pos_ok and left_ori_ok and right_ori_ok:
                break

        left_solution_deg = np.rad2deg(self.configuration.q[self.left_joint_qpos_indices])
        right_solution_deg = np.rad2deg(self.configuration.q[self.right_joint_qpos_indices])

        left_solution_deg = np.clip(
            left_solution_deg,
            self.left_joint_limits_min_deg[:left_solution_deg.shape[0]],
            self.left_joint_limits_max_deg[:left_solution_deg.shape[0]],
        )
        right_solution_deg = np.clip(
            right_solution_deg,
            self.right_joint_limits_min_deg[:right_solution_deg.shape[0]],
            self.right_joint_limits_max_deg[:right_solution_deg.shape[0]],
        )

        call_elapsed_ms = (time.perf_counter() - call_start_t) * 1000.0
        self._ik_window_calls += 1
        self._ik_window_elapsed_sum_ms += call_elapsed_ms

        window_elapsed = time.perf_counter() - self._ik_window_start_t
        if window_elapsed >= 1.0:
            avg_hz = self._ik_window_calls / max(1e-6, window_elapsed)
            avg_ms = self._ik_window_elapsed_sum_ms / max(1, self._ik_window_calls)
            logger.info(
                f"[ik] solve_dual_ik avg: freq={avg_hz:.1f}Hz, avg_time={avg_ms:.2f}ms, calls={self._ik_window_calls}"
            )
            self._ik_window_start_t = time.perf_counter()
            self._ik_window_calls = 0
            self._ik_window_elapsed_sum_ms = 0.0

        return left_solution_deg, right_solution_deg


class MinkForwardKinematics:
    """Forward kinematics using MuJoCo."""
    
    def __init__(self, xml_path: str, end_effector_site: str = "end_effector",
                 num_joints: int = 6, joint_names: list = None):
        self.xml_path = Path(xml_path)
        self.model = mujoco.MjModel.from_xml_path(str(self.xml_path))
        self.data = mujoco.MjData(self.model)
        self.end_effector_site = end_effector_site
        self.num_joints = num_joints
        if joint_names is None:
            joint_names = [f"joint{i+1}" for i in range(num_joints)]
        self.joint_names = list(joint_names[:num_joints])
        self.joint_qpos_indices = np.array(
            [self.model.joint(name).qposadr[0] for name in self.joint_names], dtype=int
        )
    
    def compute(self, joint_angles_deg: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Compute FK for given joint angles."""
        joint_angles_rad = np.deg2rad(joint_angles_deg[:self.num_joints])
        
        full_qpos = self.data.qpos.copy()
        full_qpos[self.joint_qpos_indices] = joint_angles_rad
        
        self.data.qpos[:] = full_qpos
        mujoco.mj_forward(self.model, self.data)
        
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, self.end_effector_site)
        position = self.data.site_xpos[site_id].copy()
        
        rotation_matrix = self.data.site_xmat[site_id].reshape(3, 3)
        quaternion = MinkIKSolver._mat_to_quat(rotation_matrix)
        
        return position, quaternion
