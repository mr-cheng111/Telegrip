"""
Main entry point for the unified teleoperation system.
Coordinates runtime startup, configuration, and CLI arguments.
"""

import asyncio
import argparse
import logging
import signal
import sys

from .config import TelegripConfig, load_config
from .http_api import get_local_ip
from .runtime import TelegripSystem, create_signal_handler

logger = logging.getLogger(__name__)


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description="Unified SO100 Robot Teleoperation System")

    # Control flags
    parser.add_argument("--no-robot", action="store_true", help="Disable robot connection (visualization only)")
    parser.add_argument("--no-sim", action="store_true", help="Disable simulation and inverse kinematics")
    parser.add_argument("--no-viz", action="store_true", help="Disable GUI visualization (headless mode)")
    parser.add_argument("--no-vr", action="store_true", help="Disable VR WebSocket server")
    parser.add_argument("--no-keyboard", action="store_true", help="Deprecated (keyboard control has been removed)")
    parser.add_argument("--no-https", action="store_true", help="Disable HTTPS server")
    parser.add_argument("--autoconnect", action="store_true", help="Automatically connect to robot motors on startup")
    parser.add_argument("--log-level", default="info",
                       choices=["debug", "info", "warning", "error", "critical"],
                       help="Set logging level (default: info)")

    # Network settings
    parser.add_argument("--https-port", type=int, default=None, help="HTTPS server port")
    parser.add_argument("--ws-port", type=int, default=None, help="WebSocket server port")
    parser.add_argument("--host", default=None, help="Host IP address")

    # Paths
    parser.add_argument("--urdf", default=None, help="Path to robot URDF file")
    parser.add_argument("--webapp", default=None, help="Path to webapp directory")
    parser.add_argument("--cert", default=None, help="Path to SSL certificate")
    parser.add_argument("--key", default=None, help="Path to SSL private key")

    # Robot settings
    parser.add_argument("--config", default="config.yaml", help="Path to config file")

    return parser.parse_args()


def create_config_from_args(args) -> TelegripConfig:
    """Create configuration object from command line arguments."""
    config_data = load_config(args.config)
    config = TelegripConfig()

    robot_cfg = config_data.get("robot", {})
    control_cfg = config_data.get("control", {})
    net_cfg = config_data.get("network", {})
    ssl_cfg = config_data.get("ssl", {})
    path_cfg = config_data.get("paths", {})
    gripper_cfg = config_data.get("gripper", {})
    ik_cfg = config_data.get("ik", {})
    gnirehtet_cfg = config_data.get("gnirehtet", {})

    keyboard_cfg = control_cfg.get("keyboard", {})
    vr_cfg = control_cfg.get("vr", {})
    mink_cfg = control_cfg.get("mink", {})

    config.send_interval = float(robot_cfg.get("send_interval", config.send_interval))
    config.vr_to_robot_scale = float(robot_cfg.get("vr_to_robot_scale", config.vr_to_robot_scale))
    config.require_state_feedback = bool(robot_cfg.get("require_state_feedback", getattr(config, "require_state_feedback", False)))
    cfg_backend = robot_cfg.get("command_backend", config.robot_command_backend)
    if cfg_backend is None:
        left_iface = str(robot_cfg.get("left_arm", {}).get("interface", "")).strip().lower()
        right_iface = str(robot_cfg.get("right_arm", {}).get("interface", "")).strip().lower()
        if left_iface in {"command_streaming", "ipc"} or right_iface in {"command_streaming", "ipc"}:
            cfg_backend = "command_streaming"
        else:
            cfg_backend = "ros2_topic"
    config.robot_command_backend = str(cfg_backend).strip().lower()
    arm_controller_cfg = robot_cfg.get("arm_controller", {})
    config.arm_controller_workspace = str(
        arm_controller_cfg.get("workspace", config.arm_controller_workspace)
    )
    config.arm_controller_module_dir = str(
        arm_controller_cfg.get("module_dir", config.arm_controller_module_dir)
    )
    config.arm_controller_left_mapping = str(
        arm_controller_cfg.get("left_mapping", config.arm_controller_left_mapping)
    )
    config.arm_controller_right_mapping = str(
        arm_controller_cfg.get("right_mapping", config.arm_controller_right_mapping)
    )
    config.arm_command_interpolation_alpha = float(
        arm_controller_cfg.get("interpolation_alpha", config.arm_command_interpolation_alpha)
    )
    config.arm_command_max_step_deg = float(
        arm_controller_cfg.get("max_step_deg", config.arm_command_max_step_deg)
    )
    ros2_cfg = robot_cfg.get("ros2", {})
    config.ros2_joint_state_topic = str(
        ros2_cfg.get("joint_state_topic", config.ros2_joint_state_topic)
    )
    config.ros2_left_joint_cmd_topic = str(
        ros2_cfg.get("left_joint_cmd_topic", config.ros2_left_joint_cmd_topic)
    )
    config.ros2_right_joint_cmd_topic = str(
        ros2_cfg.get("right_joint_cmd_topic", config.ros2_right_joint_cmd_topic)
    )
    config.ros2_aggregate_joint_cmd_topic = str(
        ros2_cfg.get("aggregate_joint_cmd_topic", config.ros2_aggregate_joint_cmd_topic)
    )
    config.enable_robot = bool(robot_cfg.get("left_arm", {}).get("enabled", True) or robot_cfg.get("right_arm", {}).get("enabled", True))

    config.https_port = int(net_cfg.get("https_port", config.https_port))
    config.websocket_port = int(net_cfg.get("websocket_port", config.websocket_port))
    config.host_ip = str(net_cfg.get("host_ip", config.host_ip))

    config.certfile = str(ssl_cfg.get("certfile", config.certfile))
    config.keyfile = str(ssl_cfg.get("keyfile", config.keyfile))
    config.urdf_path = str(path_cfg.get("urdf_path", config.urdf_path))
    config.webapp_dir = str(path_cfg.get("webapp_dir", config.webapp_dir))

    config.enable_keyboard = False
    config.enable_vr = bool(vr_cfg.get("enabled", config.enable_vr))
    config.vr_orientation_reference_mode = str(
        vr_cfg.get("orientation_reference_mode", config.vr_orientation_reference_mode)
    )
    rr_axis_map = vr_cfg.get("relative_rotation_axis_map", config.vr_relative_rotation_axis_map)
    config.vr_relative_rotation_axis_map = rr_axis_map if isinstance(rr_axis_map, list) else []
    config.enable_gui = bool(control_cfg.get("enable_gui", config.enable_gui))
    config.enable_sim = bool(control_cfg.get("enable_sim", config.enable_sim))
    # If simulation is disabled, GUI must also be disabled.
    if not config.enable_sim:
        config.enable_gui = False
    config.use_mink = bool(control_cfg.get("use_mink", config.use_mink))
    config.mink_mujoco_scene = str(mink_cfg.get("mujoco_scene", config.mink_mujoco_scene))
    config.end_effector_site = str(mink_cfg.get("end_effector_site", config.end_effector_site))
    config.mink_position_cost = float(mink_cfg.get("position_cost", config.mink_position_cost))
    config.mink_orientation_cost = float(mink_cfg.get("orientation_cost", config.mink_orientation_cost))
    config.mink_posture_cost = float(mink_cfg.get("posture_cost", config.mink_posture_cost))
    config.mink_lm_damping = float(mink_cfg.get("lm_damping", config.mink_lm_damping))
    config.mink_solve_damping = float(mink_cfg.get("solve_damping", config.mink_solve_damping))
    config.mink_dt = float(mink_cfg.get("dt", config.mink_dt))
    config.mink_max_iters = int(mink_cfg.get("max_iters", config.mink_max_iters))
    config.mink_position_error_threshold = float(
        mink_cfg.get("position_error_threshold", config.mink_position_error_threshold)
    )
    config.mink_orientation_error_threshold = float(
        mink_cfg.get("orientation_error_threshold", config.mink_orientation_error_threshold)
    )
    config.pos_step = float(keyboard_cfg.get("pos_step", config.pos_step))
    config.angle_step = float(keyboard_cfg.get("angle_step", config.angle_step))
    config.gripper_step = float(keyboard_cfg.get("gripper_step", config.gripper_step))

    config.gripper_open_angle = float(gripper_cfg.get("open_angle", config.gripper_open_angle))
    config.gripper_closed_angle = float(gripper_cfg.get("closed_angle", config.gripper_closed_angle))

    config.use_reference_poses = bool(ik_cfg.get("use_reference_poses", config.use_reference_poses))
    config.reference_poses_file = str(ik_cfg.get("reference_poses_file", config.reference_poses_file))
    config.ik_position_error_threshold = float(ik_cfg.get("position_error_threshold", config.ik_position_error_threshold))
    config.ik_hysteresis_threshold = float(ik_cfg.get("hysteresis_threshold", config.ik_hysteresis_threshold))
    config.ik_movement_penalty_weight = float(ik_cfg.get("movement_penalty_weight", config.ik_movement_penalty_weight))
    config.gnirehtet_enabled = bool(gnirehtet_cfg.get("enabled", config.gnirehtet_enabled))
    config.gnirehtet_binary = str(gnirehtet_cfg.get("binary", config.gnirehtet_binary))
    config.gnirehtet_mode = str(gnirehtet_cfg.get("mode", config.gnirehtet_mode))
    config.gnirehtet_args = [str(arg) for arg in gnirehtet_cfg.get("args", config.gnirehtet_args or [])]

    if args.no_robot:
        config.enable_robot = False
    if args.no_sim:
        config.enable_sim = False
        config.enable_gui = False
    elif args.no_viz:
        config.enable_gui = False
    if args.no_vr:
        config.enable_vr = False
    if args.no_keyboard:
        config.enable_keyboard = False
    config.autoconnect = args.autoconnect
    config.log_level = args.log_level

    if args.https_port is not None:
        config.https_port = args.https_port
    if args.ws_port is not None:
        config.websocket_port = args.ws_port
    if args.host is not None:
        config.host_ip = args.host

    if args.urdf is not None:
        config.urdf_path = args.urdf
    if args.webapp is not None:
        config.webapp_dir = args.webapp
    if args.cert is not None:
        config.certfile = args.cert
    if args.key is not None:
        config.keyfile = args.key

    return config


async def main():
    """Main entry point."""
    args = parse_arguments()
    log_level = getattr(logging, args.log_level.upper())

    if log_level <= logging.INFO:
        logging.basicConfig(
            level=log_level,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
    else:
        logging.basicConfig(
            level=log_level,
            format='%(message)s'
        )

    logging.getLogger('websockets').setLevel(logging.WARNING)

    config = create_config_from_args(args)

    if not config.ensure_ssl_certificates():
        logger.error("Failed to ensure SSL certificates are available")
        sys.exit(1)

    if log_level <= logging.INFO:
        logger.info("Starting with configuration:")
        logger.info(f"  Robot: {'enabled' if config.enable_robot else 'disabled'}")
        logger.info(f"  Simulation: {'enabled' if config.enable_sim else 'disabled'}")
        logger.info(f"  Headless mode: {'enabled' if not config.enable_gui and config.enable_sim else 'disabled'}")
        logger.info(f"  VR: {'enabled' if config.enable_vr else 'disabled'}")
        logger.info("  Keyboard: removed")
        logger.info(f"  Auto-connect: {'enabled' if config.autoconnect else 'disabled'}")
        logger.info(f"  Gnirehtet auto-start: {'enabled' if config.gnirehtet_enabled else 'disabled'}")
        logger.info(f"  HTTPS Port: {config.https_port}")
        logger.info(f"  WebSocket Port: {config.websocket_port}")
    else:
        host_display = get_local_ip() if config.host_ip == "0.0.0.0" else config.host_ip
        print(f"🤖 telegrip starting...")
        print(f"📱 Open the UI in your browser on:")
        print(f"   https://{host_display}:{config.https_port}")
        print(f"📱 Then go to the same address on your VR headset browser")
        print(f"💡 Use --log-level info to see detailed output")
        print()

    system = TelegripSystem(config)

    loop = asyncio.get_event_loop()
    signal_handler = create_signal_handler(system, loop)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        await system.start()
    except (KeyboardInterrupt, SystemExit):
        if log_level <= logging.INFO:
            logger.info("Received interrupt signal")
        else:
            print("\n🛑 Shutting down...")
    except asyncio.CancelledError:
        if log_level <= logging.INFO:
            logger.info("System tasks cancelled")
    except Exception as e:
        if log_level <= logging.INFO:
            logger.error(f"System error: {e}")
        else:
            print(f"❌ Error: {e}")
    finally:
        try:
            await system.stop()
        except (asyncio.CancelledError, SystemExit):
            pass

        def ignore_ssl_errors(loop, context):
            if 'exception' in context:
                exc = context['exception']
                if isinstance(exc, (OSError, RuntimeError)):
                    return
            loop.default_exception_handler(context)

        loop.set_exception_handler(ignore_ssl_errors)

        if log_level > logging.INFO:
            print("✅ Shutdown complete.")


def main_cli():
    """Console script entry point for pip-installed package."""
    try:
        asyncio.run(main())
    except (KeyboardInterrupt, SystemExit):
        print("\nShutdown complete.")
    except asyncio.CancelledError:
        pass
    except Exception as e:
        logger.error(f"Fatal error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main_cli()
