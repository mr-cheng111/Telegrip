"""
Centralized teleoperation frame-mapping utilities.

This module gathers all telegrip-specific frame conventions in one place:
- VR relative translation -> robot target translation
- controller quaternion delta -> end-effector target quaternion
- optional post axis remap for rotation deltas
- fixed end-effector orientation correction
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass
class TeleopFrameMapperConfig:
    """集中描述 telegrip 当前使用的坐标/姿态补偿参数。"""

    translation_euler_xyz_deg: tuple[float, float, float] = (0.0, 0.0, 180.0)
    controller_delta_to_target_axis_map: np.ndarray = field(
        default_factory=lambda: np.eye(3, dtype=float)
    )
    ee_target_orientation_correction_euler_xyz_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)


class TeleopFrameMapper:
    """
    统一处理 telegrip 中所有“设计性旋转”。

    公式约定:
    1. 平移映射:
       delta_target = R_translation * delta_vr

    2. 姿态增量映射:
       R_delta_target = S * R_delta_hand * S^T
       R_delta_target = U * R_delta_target * U^T

    3. 目标姿态合成:
       q_target = (q_origin_target * q_ee_fix) * q_delta_target

    4. 当启用“R_fix 驱动 U 自动更新”时：
       不再直接依赖手工维护的 U_calib，而是按语义动态求解 U_eff。

       定义：
       - right = -Y_base
       - back  = -X_base
       - down  = -Z_base

       令：
       - R_base: base_link 在世界系下的旋转
       - R_fix:  ee_target_orientation_correction 对应旋转
       - R_ref = R_base * R_fix

       则最终目标局部坐标系在世界系下的三个轴方向为 R_ref 的三列。
       我们希望手柄语义轴满足：
       - X_hand -> target 中“朝右”的那根轴
       - Y_hand -> target 中“朝后”的那根轴
       - Z_hand -> target 中“朝下”的那根轴

       因此先将这些语义方向写成世界向量，再投回 target 局部系：
       - u_x = R_ref^T * v_right_world
       - u_y = R_ref^T * v_back_world
       - u_z = R_ref^T * v_down_world

       最终：
       U_eff = [u_x, u_y, u_z]
    """

    def __init__(self, config: TeleopFrameMapperConfig):
        self.config = config
        self.translation_quat_wxyz = self.quat_from_euler_xyz_deg(
            *config.translation_euler_xyz_deg
        )
        self.ee_target_orientation_correction_quat_wxyz = self.quat_from_euler_xyz_deg(
            *config.ee_target_orientation_correction_euler_xyz_deg
        )
    @classmethod
    def from_telegrip_config(cls, telegrip_config) -> "TeleopFrameMapper":
        """
        从 TelegripConfig 构造统一的旋转/平移映射器。

        这里是全链路唯一的“设计性姿态补偿”入口。
        后续若需调试：
        1. VR 相对平移到机器人基坐标的映射
        2. 手柄相对旋转的轴交换/翻转
        3. 末端固定姿态补偿
        只修改这里即可。
        """
        translation_euler_xyz_deg = tuple(
            float(v)
            for v in getattr(
                telegrip_config,
                "teleop_frame_translation_euler_xyz_deg",
                [0.0, 0.0, 180.0],
            )[:3]
        )
        ee_target_orientation_correction_euler_xyz_deg = tuple(
            float(v)
            for v in getattr(
                telegrip_config,
                "teleop_frame_ee_target_orientation_correction_euler_xyz_deg",
                [0.0, 0.0, 0.0],
            )[:3]
        )
        return cls(
            TeleopFrameMapperConfig(
                translation_euler_xyz_deg=translation_euler_xyz_deg,
                controller_delta_to_target_axis_map=np.eye(3, dtype=float),
                ee_target_orientation_correction_euler_xyz_deg=ee_target_orientation_correction_euler_xyz_deg,
            )
        )

    def get_effective_relative_rotation_axis_map(
        self,
        origin_target_quat_wxyz: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """
        返回当前运行时实际生效的 U_eff。

        这里固定根据当前 base_link 姿态与 ee 修正，
        按 right/back/down 语义动态计算 U_eff。
        """
        if origin_target_quat_wxyz is None:
            return np.eye(3, dtype=float)
        return self._build_semantic_relative_rotation_axis_map(origin_target_quat_wxyz)

    def _build_semantic_relative_rotation_axis_map(
        self,
        origin_target_quat_wxyz: np.ndarray,
    ) -> np.ndarray:
        """
        基于 base_link 语义方向动态构造 U_eff。

        约定:
        - right = -Y_base
        - back  = -X_base
        - down  = -Z_base

        数学上：
        1. R_base = Rot(q_origin)
        2. R_fix  = Rot(q_ee_fix)
        3. R_ref  = R_base * R_fix
        4. 语义世界方向:
           v_right = R_base * [ 0, -1,  0]^T
           v_back  = R_base * [-1,  0,  0]^T
           v_down  = R_base * [ 0,  0, -1]^T
        5. 投回 target 局部系:
           u_i = R_ref^T * v_i

        因为 R_ref 的列向量就是 target 局部 XYZ 轴在世界系下的朝向，
        所以 R_ref^T * v_i 表示“语义方向 v_i 在 target 局部坐标中的表达”。
        将三个结果作为列拼起来，就得到手柄局部轴到 target 局部轴的映射 U_eff。
        """
        q_origin = self.normalize_wxyz(origin_target_quat_wxyz)
        R_base = self.quat_to_rotation_matrix_wxyz(q_origin)
        R_fix = self.quat_to_rotation_matrix_wxyz(self.ee_target_orientation_correction_quat_wxyz)
        R_ref = R_base @ R_fix

        v_right_world = R_base @ np.array([0.0, -1.0, 0.0], dtype=float)
        v_back_world = R_base @ np.array([-1.0, 0.0, 0.0], dtype=float)
        v_down_world = R_base @ np.array([0.0, 0.0, -1.0], dtype=float)

        u_x = R_ref.T @ v_right_world
        u_y = R_ref.T @ v_back_world
        u_z = R_ref.T @ v_down_world
        U_eff = np.column_stack((u_x, u_y, u_z))

        # 数值误差下重新正交化，确保后续 U * R * U^T 仍是标准旋转变换。
        u, _, vh = np.linalg.svd(U_eff)
        U_eff_ortho = u @ vh
        if np.linalg.det(U_eff_ortho) < 0.0:
            u[:, -1] *= -1.0
            U_eff_ortho = u @ vh
        return U_eff_ortho

    @staticmethod
    def normalize_xyzw(q: np.ndarray) -> Optional[np.ndarray]:
        arr = np.asarray(q, dtype=float).reshape(-1)[:4]
        n = np.linalg.norm(arr)
        if n <= 1e-12:
            return None
        return arr / n

    @staticmethod
    def xyzw_to_wxyz(q_xyzw: np.ndarray) -> np.ndarray:
        q = np.asarray(q_xyzw, dtype=float).reshape(-1)[:4]
        return np.array([q[3], q[0], q[1], q[2]], dtype=float)

    @staticmethod
    def normalize_wxyz(q: np.ndarray) -> np.ndarray:
        arr = np.asarray(q, dtype=float).reshape(-1)[:4]
        n = np.linalg.norm(arr)
        if n <= 1e-12:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        return arr / n

    @staticmethod
    def quat_multiply_wxyz(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array(
            [
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            ],
            dtype=float,
        )

    @staticmethod
    def quat_inverse_wxyz(q: np.ndarray) -> np.ndarray:
        w, x, y, z = q
        n2 = w * w + x * x + y * y + z * z
        if n2 <= 1e-12:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        return np.array([w, -x, -y, -z], dtype=float) / n2

    @staticmethod
    def quat_to_rotation_matrix_wxyz(q: np.ndarray) -> np.ndarray:
        w, x, y, z = TeleopFrameMapper.normalize_wxyz(q)
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
    def quat_from_rotation_matrix_wxyz(R: np.ndarray) -> np.ndarray:
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
        return TeleopFrameMapper.normalize_wxyz(np.array([w, x, y, z], dtype=float))

    @staticmethod
    def quat_from_euler_xyz_deg(rx_deg: float, ry_deg: float, rz_deg: float) -> np.ndarray:
        rx = np.deg2rad(rx_deg)
        ry = np.deg2rad(ry_deg)
        rz = np.deg2rad(rz_deg)
        cx, sx = np.cos(rx), np.sin(rx)
        cy, sy = np.cos(ry), np.sin(ry)
        cz, sz = np.cos(rz), np.sin(rz)
        Rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]], dtype=float)
        Ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]], dtype=float)
        Rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]], dtype=float)
        return TeleopFrameMapper.quat_from_rotation_matrix_wxyz(Rz @ Ry @ Rx)

    @staticmethod
    def rotate_vec_by_quat_wxyz(vec: np.ndarray, quat_wxyz: np.ndarray) -> np.ndarray:
        v = np.asarray(vec, dtype=float).reshape(-1)[:3]
        q = TeleopFrameMapper.normalize_wxyz(quat_wxyz)
        vq = np.array([0.0, v[0], v[1], v[2]], dtype=float)
        qv = TeleopFrameMapper.quat_multiply_wxyz(q, vq)
        qvq = TeleopFrameMapper.quat_multiply_wxyz(qv, TeleopFrameMapper.quat_inverse_wxyz(q))
        return qvq[1:4].copy()

    def map_relative_translation(self, delta_vr: np.ndarray) -> np.ndarray:
        return self.rotate_vec_by_quat_wxyz(delta_vr, self.translation_quat_wxyz)

    def build_relative_controller_delta_wxyz(
        self, current_controller_xyzw: np.ndarray, reference_controller_xyzw: np.ndarray
    ) -> Optional[np.ndarray]:
        current_xyzw = self.normalize_xyzw(current_controller_xyzw)
        reference_xyzw = self.normalize_xyzw(reference_controller_xyzw)
        if current_xyzw is None or reference_xyzw is None:
            return None

        from scipy.spatial.transform import Rotation as R

        rel_xyzw = (
            R.from_quat(reference_xyzw).inv()
            * R.from_quat(current_xyzw)
        ).as_quat()
        return self.xyzw_to_wxyz(rel_xyzw)

    def map_target_orientation(
        self,
        controller_delta_quat_wxyz: np.ndarray,
        origin_target_quat_wxyz: np.ndarray,
    ) -> np.ndarray:
        q_delta_hand = self.normalize_wxyz(controller_delta_quat_wxyz)
        q_origin = self.normalize_wxyz(origin_target_quat_wxyz)

        R_delta_hand = self.quat_to_rotation_matrix_wxyz(q_delta_hand)
        S = self.config.controller_delta_to_target_axis_map
        U = self.get_effective_relative_rotation_axis_map(q_origin)
        R_delta_target = S @ R_delta_hand @ S.T
        R_delta_target = U @ R_delta_target @ U.T
        q_delta_target = self.quat_from_rotation_matrix_wxyz(R_delta_target)

        # 将 q_ee_fix 并入参考姿态：
        #   q_ref_aligned = q_origin_target * q_ee_fix
        #   q_target      = q_ref_aligned * q_delta_target
        #
        # 这样 q_ee_fix 只负责修正“初始静态朝向”，
        # 不再改变后续 q_delta_target 的 XYZ 轴语义。
        q_ref_aligned = self.quat_multiply_wxyz(
            q_origin, self.ee_target_orientation_correction_quat_wxyz
        )
        q_target = self.quat_multiply_wxyz(q_ref_aligned, q_delta_target)
        return self.normalize_wxyz(q_target)
