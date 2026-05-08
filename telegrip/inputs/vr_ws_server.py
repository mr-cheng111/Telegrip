"""
VR WebSocket server for receiving controller data from web browsers.
Adapted from the original vr_robot_teleop.py script.
"""

import asyncio
import json
import ssl
import time
import websockets
import numpy as np
import logging
from typing import Dict, Optional, Set

from .base import BaseInputProvider, ControlGoal, ControlMode
from ..config import TelegripConfig, get_config_data, update_config_data
from ..core.teleop_frame_mapper import TeleopFrameMapper
from ..core.kinematics import compute_relative_position

logger = logging.getLogger(__name__)


class VRControllerState:
    """State tracking for a VR controller."""
    
    def __init__(self, hand: str):
        self.hand = hand
        self.grip_active = False
        self.trigger_active = False
        
        # Position tracking for relative movement
        self.origin_position = None
        self.origin_quaternion = None
        self.current_position = None
    
    def reset_grip(self):
        """Reset grip state but preserve trigger state."""
        self.grip_active = False
        self.origin_position = None
        self.origin_quaternion = None


class VRWebSocketServer(BaseInputProvider):
    """WebSocket server for VR controller input."""
    
    def __init__(self, command_queue: asyncio.Queue, config: TelegripConfig):
        super().__init__(command_queue)
        self.config = config
        self.clients: Set = set()
        self.server = None
        
        # Controller states
        self.left_controller = VRControllerState("left")
        self.right_controller = VRControllerState("right")
        
        # Robot state tracking (for relative position calculation)
        self.left_arm_origin_position = None
        self.right_arm_origin_position = None
        self.orientation_reference_mode = self._normalize_orientation_reference_mode(
            getattr(self.config, "vr_orientation_reference_mode", "global_calibration")
        )
        # Global orientation calibration references in controller quaternion order [x, y, z, w].
        self.hand_orientation_ref_xyzw = {"left": None, "right": None}
        self._load_orientation_refs_from_config()
        self._calib_hold_start_ts = None
        self._calib_hold_last_active_ts = None
        self._calib_hold_done = False
        self._motion_debug_last_log_ts = {"left": 0.0, "right": 0.0}
        self.frame_mapper = TeleopFrameMapper.from_telegrip_config(self.config)
        logger.info(f"VR orientation reference mode: {self.orientation_reference_mode}")

    @staticmethod
    def _quat_xyzw_to_euler_xyz_deg(q_xyzw: np.ndarray) -> Optional[np.ndarray]:
        """
        将手柄四元数 [x, y, z, w] 转成 XYZ 欧拉角（单位：度）。

        公式来源：
          1. 先由四元数得到旋转矩阵 R
          2. 按 XYZ 固定轴分解，满足：
               R = Rz(rz) * Ry(ry) * Rx(rx)
        """
        q_norm = TeleopFrameMapper.normalize_xyzw(q_xyzw)
        if q_norm is None:
            return None
        q_wxyz = TeleopFrameMapper.xyzw_to_wxyz(q_norm)
        R = TeleopFrameMapper.quat_to_rotation_matrix_wxyz(q_wxyz)

        sy = float(np.clip(R[0, 2], -1.0, 1.0))
        ry = np.arcsin(sy)
        cy = np.cos(ry)

        if abs(cy) > 1e-8:
            rx = np.arctan2(-R[1, 2], R[2, 2])
            rz = np.arctan2(-R[0, 1], R[0, 0])
        else:
            # 万向节附近退化为一组稳定解，固定 rz=0。
            rx = np.arctan2(R[2, 1], R[1, 1])
            rz = 0.0

        return np.rad2deg(np.array([rx, ry, rz], dtype=float))

    def _log_controller_reference_pose(self, hand: str, q_xyzw: np.ndarray, reason: str):
        """把当前手柄参考姿态直接输出到日志，方便调坐标系。"""
        q_norm = self._normalize_xyzw(np.asarray(q_xyzw, dtype=float))
        if q_norm is None:
            logger.warning(f"{hand.upper()} controller reference pose unavailable: invalid quaternion")
            return
        euler_xyz_deg = self._quat_xyzw_to_euler_xyz_deg(q_norm)
        if euler_xyz_deg is None:
            logger.info(
                f"{hand.upper()} controller reference pose ({reason}): "
                f"quat_xyzw={np.round(q_norm, 6)}"
            )
            return
        logger.info(
            f"{hand.upper()} controller reference pose ({reason}): "
            f"quat_xyzw={np.round(q_norm, 6)}, "
            f"euler_xyz_deg={np.round(euler_xyz_deg, 2)}"
        )

    @staticmethod
    def _normalize_xyzw(q: np.ndarray) -> np.ndarray:
        return TeleopFrameMapper.normalize_xyzw(q)

    @staticmethod
    def _normalize_orientation_reference_mode(mode: str) -> str:
        raw = str(mode).strip().lower()
        if raw in ("global", "global_calibration", "calibration", "config"):
            return "global_calibration"
        if raw in ("grip", "grip_press", "grip_down", "grip-origin", "grip_origin"):
            return "grip_press"
        return "global_calibration"

    def _resolve_orientation_reference_xyzw(self, hand: str, controller: VRControllerState) -> Optional[np.ndarray]:
        """Return orientation reference according to configured mode."""
        if self.orientation_reference_mode == "grip_press":
            if controller.origin_quaternion is None:
                return None
            return self._normalize_xyzw(np.asarray(controller.origin_quaternion, dtype=float))
        return self.hand_orientation_ref_xyzw.get(hand)

    def _load_orientation_refs_from_config(self):
        """Load saved orientation reference quaternions from config.yaml."""
        try:
            cfg = get_config_data()
            refs = (
                cfg.get("control", {})
                .get("vr", {})
                .get("orientation_reference_quat_xyzw", {})
            )
            for hand in ("left", "right"):
                q = refs.get(hand)
                if isinstance(q, list) and len(q) >= 4:
                    nq = self._normalize_xyzw(np.array(q[:4], dtype=float))
                    if nq is not None:
                        self.hand_orientation_ref_xyzw[hand] = nq
                        self._log_controller_reference_pose(hand, nq, "loaded_from_config")
            if self.hand_orientation_ref_xyzw["left"] is not None and self.hand_orientation_ref_xyzw["right"] is not None:
                logger.info("Loaded VR orientation references from config.yaml")
        except Exception as e:
            logger.warning(f"Could not load VR orientation references from config.yaml: {e}")

    def _save_orientation_refs_to_config(self):
        """Persist orientation reference quaternions into config.yaml."""
        cfg = get_config_data()
        control = cfg.setdefault("control", {})
        vr = control.setdefault("vr", {})
        vr["orientation_reference_quat_xyzw"] = {
            "left": self.hand_orientation_ref_xyzw["left"].tolist(),
            "right": self.hand_orientation_ref_xyzw["right"].tolist(),
        }
        update_config_data(cfg)

    def _set_orientation_refs_from_xyzw(self, left_xyzw: np.ndarray, right_xyzw: np.ndarray, reason: str):
        """用当前左右手柄姿态直接更新姿态参考零点。"""
        left_norm = self._normalize_xyzw(np.asarray(left_xyzw, dtype=float))
        right_norm = self._normalize_xyzw(np.asarray(right_xyzw, dtype=float))
        if left_norm is None or right_norm is None:
            logger.warning("Could not update orientation references: invalid controller quaternion")
            return
        self.hand_orientation_ref_xyzw["left"] = left_norm
        self.hand_orientation_ref_xyzw["right"] = right_norm
        self._save_orientation_refs_to_config()
        self._log_controller_reference_pose("left", left_norm, reason)
        self._log_controller_reference_pose("right", right_norm, reason)
        logger.info("Updated VR orientation references for both hands")

    def _update_orientation_calibration_hold(self, left_data: Dict, right_data: Dict):
        """LEFT X + RIGHT A 长按 3 秒后，更新双手姿态参考零点。"""
        # Trigger-like value path (same style as trigger > 0.5), then fallbacks.
        lx = float(left_data.get("x", 0.0)) > 0.5
        ra = float(right_data.get("a", 0.0)) > 0.5
        if not lx:
            lx = bool(left_data.get("xButton", False))
        if not ra:
            ra = bool(right_data.get("aButton", False))
        if not lx:
            lbtn = left_data.get("buttons", [])
            if isinstance(lbtn, list):
                try:
                    lx = (
                        (len(lbtn) > 3 and float(lbtn[3]) > 0.5)
                        or (len(lbtn) > 4 and float(lbtn[4]) > 0.5)
                    )
                except Exception:
                    lx = False
        if not ra:
            rbtn = right_data.get("buttons", [])
            if isinstance(rbtn, list):
                try:
                    ra = (
                        (len(rbtn) > 3 and float(rbtn[3]) > 0.5)
                        or (len(rbtn) > 4 and float(rbtn[4]) > 0.5)
                    )
                except Exception:
                    ra = False
        lq = left_data.get("quaternion") or {}
        rq = right_data.get("quaternion") or {}
        has_lq = all(k in lq for k in ("x", "y", "z", "w"))
        has_rq = all(k in rq for k in ("x", "y", "z", "w"))
        combo_active = lx and ra and has_lq and has_rq

        now = time.time()
        # 允许按键/数据包出现极短暂抖动，不要立刻把 3 秒长按计时清零。
        # 这里给一个小的容错窗，避免 Quest/WebXR 某些帧按钮态瞬时掉到 false。
        hold_grace_sec = 0.35

        if not combo_active:
            if (
                self._calib_hold_start_ts is not None
                and self._calib_hold_last_active_ts is not None
                and (now - self._calib_hold_last_active_ts) <= hold_grace_sec
            ):
                return

            self._calib_hold_start_ts = None
            self._calib_hold_last_active_ts = None
            self._calib_hold_done = False
            return

        if self._calib_hold_start_ts is None:
            self._calib_hold_start_ts = now
            self._calib_hold_last_active_ts = now
            self._calib_hold_done = False
            print(
                "[x/a] hold started: LEFT X + RIGHT A detected, keep holding for 3s...",
                flush=True,
            )
            return

        self._calib_hold_last_active_ts = now

        if self._calib_hold_done:
            return

        if now - self._calib_hold_start_ts >= 3.0:
            left_xyzw = np.array(
                [float(lq["x"]), float(lq["y"]), float(lq["z"]), float(lq["w"])],
                dtype=float,
            )
            right_xyzw = np.array(
                [float(rq["x"]), float(rq["y"]), float(rq["z"]), float(rq["w"])],
                dtype=float,
            )
            self._set_orientation_refs_from_xyzw(
                left_xyzw,
                right_xyzw,
                reason="xa_recalibration",
            )
            try:
                recalib_goal = ControlGoal(
                    arm="left",
                    metadata={
                        "source": "vr_xa_long_hold",
                        "publish_mainpy_joint_target_now": True,
                    },
                )
                self.command_queue.put_nowait(recalib_goal)
            except Exception:
                logger.warning("Could not queue X/A long-hold publish request")
                return
            self._calib_hold_done = True
            print("✅ [x/a] updated orientation references for both hands", flush=True)

    def _get_local_ip(self) -> str:
        """Get the local IP address of this machine."""
        import socket
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))
                return s.getsockname()[0]
        except Exception:
            try:
                return socket.gethostbyname(socket.gethostname())
            except Exception:
                return "localhost"

    def setup_ssl(self) -> Optional[ssl.SSLContext]:
        """Setup SSL context for WebSocket server."""
        # Automatically generate SSL certificates if they don't exist
        if not self.config.ssl_files_exist:
            logger.info("SSL certificates not found for WebSocket server, attempting to generate them...")
            if not self.config.ensure_ssl_certificates():
                logger.error("Failed to generate SSL certificates for WebSocket server")
                return None
        
        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        try:
            # Get absolute paths for SSL certificates
            cert_path, key_path = self.config.get_absolute_ssl_paths()
            ssl_context.load_cert_chain(certfile=cert_path, keyfile=key_path)
            logger.info("SSL certificate and key loaded successfully for WebSocket server")
            return ssl_context
        except ssl.SSLError as e:
            logger.error(f"Error loading SSL cert/key: {e}")
            return None
    
    async def start(self):
        """Start the WebSocket server."""
        if not self.config.enable_vr:
            logger.info("VR WebSocket server disabled in configuration")
            return
        
        ssl_context = self.setup_ssl()
        if ssl_context is None:
            logger.error("Failed to setup SSL for WebSocket server")
            return
        
        host = self.config.host_ip
        port = self.config.websocket_port
        self._browser_warning_shown = False

        try:
            self.server = await websockets.serve(
                self.websocket_handler,
                host,
                port,
                ssl=ssl_context,
                process_request=self._process_request
            )
            self.is_running = True
            host_display = self._get_local_ip() if host == "0.0.0.0" else host
            logger.info(f"VR WebSocket server running on wss://{host_display}:{port}")
        except Exception as e:
            logger.error(f"Failed to start WebSocket server: {e}")

    async def _process_request(self, connection, request):
        """Process incoming requests and detect browser visits to the WebSocket port."""
        # Check if this looks like a browser request (not a proper WebSocket upgrade)
        # In newer websockets versions, request.headers is a Headers object
        headers = request.headers
        connection_header = headers.get("Connection", "")
        upgrade_header = headers.get("Upgrade", "")

        # Proper WebSocket requests have "Upgrade" in Connection header and "websocket" in Upgrade header
        is_websocket_request = (
            "upgrade" in connection_header.lower() and
            "websocket" in upgrade_header.lower()
        )

        if not is_websocket_request:
            # Only show warning once to avoid spam
            if not self._browser_warning_shown:
                self._browser_warning_shown = True
                host_display = self._get_local_ip() if self.config.host_ip == "0.0.0.0" else self.config.host_ip
                print(f"\n⚠️  Someone is trying to open port {self.config.websocket_port} in a browser.")
                print(f"   This port is for VR WebSocket connections only.")
                print(f"   The web UI is at: https://{host_display}:{self.config.https_port}\n")

        # Return None to let websockets library handle the request normally
        # (it will reject non-WebSocket requests with 426 Upgrade Required)
        return None

    async def stop(self):
        """Stop the WebSocket server."""
        self.is_running = False

        # Close all active client connections to unblock websocket_handler
        for client in list(self.clients):
            try:
                await client.close()
            except Exception:
                pass

        if self.server:
            self.server.close()
            await self.server.wait_closed()
            logger.info("VR WebSocket server stopped")
    
    async def websocket_handler(self, websocket, path=None):
        """Handle WebSocket connections from VR controllers."""
        client_address = websocket.remote_address
        logger.info(f"VR client connected: {client_address}")
        self.clients.add(websocket)
        
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    await self.process_controller_data(data)
                except json.JSONDecodeError:
                    logger.warning(f"Received non-JSON message: {message}")
                except Exception as e:
                    logger.error(f"Error processing VR data: {e}")
        
        except websockets.exceptions.ConnectionClosedOK:
            logger.info(f"VR client {client_address} disconnected normally")
        except websockets.exceptions.ConnectionClosedError as e:
            logger.warning(f"VR client {client_address} disconnected with error: {e}")
        except Exception as e:
            logger.error(f"Unexpected error with VR client {client_address}: {e}")
        finally:
            self.clients.discard(websocket)
            # Handle grip releases when client disconnects
            await self.handle_grip_release('left')
            await self.handle_grip_release('right')
            logger.info(f"VR client {client_address} cleanup complete")
    
    async def process_controller_data(self, data: Dict):
        """Process incoming VR controller data."""
        
        # Primary path: dual controller packet from web-ui/vr_app.js
        if 'leftController' in data and 'rightController' in data:
            left_data = data['leftController']
            right_data = data['rightController']
            if not isinstance(left_data, dict) or not isinstance(right_data, dict):
                return
            self._update_orientation_calibration_hold(left_data, right_data)
            
            # Always process trigger state (gripper), independent of grip.
            await self.process_single_controller('left', left_data)
            await self.process_single_controller('right', right_data)

            # Grip exclusively controls arm motion mode.
            if not left_data.get('gripActive', False) and self.left_controller.grip_active:
                await self.handle_grip_release('left')
            
            if not right_data.get('gripActive', False) and self.right_controller.grip_active:
                await self.handle_grip_release('right')
                
            return

        # Legacy single controller control packets are no longer supported.
        # Keep only explicit release compatibility.
        hand = data.get('hand')
        
        # Handle explicit release messages
        if data.get('gripReleased'):
            await self.handle_grip_release(hand)
            return
        
        if data.get('triggerReleased'):
            await self.handle_trigger_release(hand)
            return
    
    async def process_single_controller(self, hand: str, data: Dict):
        """Process data for a single controller."""
        position = data.get('position', {})
        quaternion = data.get('quaternion', {})  # Get quaternion data directly
        grip_active = data.get('gripActive', False)
        trigger = data.get('trigger', 0)
        
        controller = self.left_controller if hand == 'left' else self.right_controller
        
        # Handle trigger for gripper control
        trigger_active = trigger > 0.5
        if trigger_active != controller.trigger_active:
            controller.trigger_active = trigger_active
            
            # Send gripper control goal - do not specify mode to avoid interfering with position control
            # Trigger pressed -> close gripper; trigger released -> open gripper.
            gripper_goal = ControlGoal(
                arm=hand,
                gripper_closed=trigger_active,
                metadata={"source": "vr_trigger"}
            )
            await self.send_goal(gripper_goal)
            
            logger.info(f"🤏 {hand.upper()} gripper {'CLOSED' if trigger_active else 'OPENED'}")
        
        # Handle grip button for arm movement control
        if grip_active and position:
            if not controller.grip_active:
                # Grip just activated - set origin and reset target position
                controller.grip_active = True
                controller.origin_position = position.copy()

                # 当前输入链路只保留 quaternion 姿态，不再兼容旧的 Euler 回退路径。
                if quaternion and all(k in quaternion for k in ['x', 'y', 'z', 'w']):
                    controller.origin_quaternion = np.array(
                        [quaternion['x'], quaternion['y'], quaternion['z'], quaternion['w']],
                        dtype=float,
                    )
                    self._log_controller_reference_pose(hand, controller.origin_quaternion, "grip_origin")
                else:
                    controller.origin_quaternion = None
                
                # Send reset signal to control loop to reset target position to current robot position
                reset_goal = ControlGoal(
                    arm=hand,
                    mode=ControlMode.POSITION_CONTROL,  # Keep in position control
                    target_position=None,  # Special signal
                    metadata={
                        "source": f"vr_grip_reset_{hand}",
                        "reset_target_to_current": True  # Signal to reset target to current position
                    }
                )
                await self.send_goal(reset_goal)
                
                logger.info(f"🔒 {hand.upper()} grip activated - controlling {hand} arm (target reset to current position)")
            
            # Compute target position
            if controller.origin_position:
                relative_delta = compute_relative_position(
                    position, 
                    controller.origin_position, 
                    self.config.vr_to_robot_scale
                )
                now = time.time()
                if now - self._motion_debug_last_log_ts[hand] >= 1.0:
                    self._motion_debug_last_log_ts[hand] = now
                    logger.info(
                        f"🎯 {hand.upper()} VR delta: {np.round(relative_delta, 4)} m | "
                        f"norm={float(np.linalg.norm(relative_delta)):.4f}"
                    )

                marker_quat_wxyz = None
                if quaternion and all(k in quaternion for k in ['x', 'y', 'z', 'w']):
                    current_xyzw = self._normalize_xyzw(np.array([
                        float(quaternion['x']),
                        float(quaternion['y']),
                        float(quaternion['z']),
                        float(quaternion['w']),
                    ], dtype=float))
                    if current_xyzw is not None:
                        ref_xyzw = self._resolve_orientation_reference_xyzw(hand, controller)
                        if ref_xyzw is not None:
                            # Controller local delta from configured reference pose:
                            # q_delta = inv(q_ref) * q_now  (xyzw)
                            # q_ref comes from global calibration or grip-press origin.
                            marker_quat_wxyz = self.frame_mapper.build_relative_controller_delta_wxyz(
                                current_controller_xyzw=current_xyzw,
                                reference_controller_xyzw=ref_xyzw,
                            )
                        else:
                            # Fallback when no reference pose is available yet.
                            marker_quat_wxyz = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

                goal = ControlGoal(
                    arm=hand,
                    mode=ControlMode.POSITION_CONTROL,
                    # Original telegrip style: relative translation + wrist angles only.
                    target_position=relative_delta,
                    # 恢复手柄姿态控制：这里发送的是相对参考姿态的四元数增量，
                    # 控制环内再按
                    #   q_target = q_origin_target * q_delta
                    # 合成为目标末端姿态。
                    target_orientation_quat=marker_quat_wxyz,
                    wrist_roll_deg=None,
                    wrist_flex_deg=None,
                    metadata={
                        "source": "vr_grip",
                        "relative_position": True,
                        "origin_position": controller.origin_position.copy(),
                        "marker_grab_drag": True,
                    }
                )
                await self.send_goal(goal)
    
    async def handle_grip_release(self, hand: str):
        """Handle grip release for a controller."""
        if hand == 'left':
            controller = self.left_controller
        elif hand == 'right':
            controller = self.right_controller
        else:
            return
        
        if controller.grip_active:
            controller.reset_grip()
            
            # Send idle goal to stop arm control
            goal = ControlGoal(
                arm=hand,
                mode=ControlMode.IDLE,
                metadata={"source": "vr_grip_release"}
            )
            await self.send_goal(goal)
            
            logger.info(f"🔓 {hand.upper()} grip released - arm control stopped")
    
    async def handle_trigger_release(self, hand: str):
        """Handle trigger release for a controller."""
        controller = self.left_controller if hand == 'left' else self.right_controller
        
        if controller.trigger_active:
            controller.trigger_active = False
            
            # Trigger released -> open gripper.
            goal = ControlGoal(
                arm=hand,
                gripper_closed=False,
                metadata={"source": "vr_trigger_release"}
            )
            await self.send_goal(goal)
            
            logger.info(f"🤏 {hand.upper()} gripper OPENED (trigger released)")
