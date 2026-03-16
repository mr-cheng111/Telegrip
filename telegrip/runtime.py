"""
Runtime orchestration for telegrip system lifecycle.
"""

import asyncio
import logging
import queue
import threading

from .config import TelegripConfig
from .control_loop import ControlLoop
from .http_api import HTTPSServer
from .inputs.vr_ws_server import VRWebSocketServer
from .inputs.web_keyboard import WebKeyboardHandler

logger = logging.getLogger(__name__)


class TelegripSystem:
    """Main teleoperation system that coordinates all components."""

    def __init__(self, config: TelegripConfig):
        self.config = config

        self.command_queue = asyncio.Queue()
        self.control_commands_queue = queue.Queue(maxsize=10)

        self.https_server = HTTPSServer(config)
        self.vr_server = VRWebSocketServer(self.command_queue, config)
        self.web_keyboard_handler = WebKeyboardHandler(self.command_queue, config)
        self.control_loop = ControlLoop(self.command_queue, config, self.control_commands_queue)

        self.https_server.set_system_ref(self)

        self.control_loop.web_keyboard_handler = self.web_keyboard_handler
        self.web_keyboard_handler.disconnect_callback = lambda: self.add_control_command("robot_disconnect")

        self.tasks = []
        self.is_running = False
        self.main_loop = None

    def add_control_command(self, action: str):
        """Add a control command to the queue for processing."""
        try:
            command = {"action": action}
            logger.info(f"🔌 Queueing control command: {command}")
            self.control_commands_queue.put_nowait(command)
            logger.info("🔌 Command queued successfully")
        except queue.Full:
            logger.warning(f"Control commands queue is full, dropping command: {action}")
        except Exception as e:
            logger.error(f"🔌 Error queuing command: {e}")

    def add_keypress_command(self, command: dict):
        """Add a keypress command to the queue for processing."""
        try:
            logger.info(f"🎮 Queueing keypress command: {command}")
            self.control_commands_queue.put_nowait(command)
            logger.info("🎮 Keypress command queued successfully")
        except queue.Full:
            logger.warning(f"Control commands queue is full, dropping keypress command: {command}")
        except Exception as e:
            logger.error(f"🎮 Error queuing keypress command: {e}")

    async def process_control_commands(self):
        """Process control commands from the thread-safe queue."""
        try:
            commands_to_process = []
            while True:
                try:
                    command = self.control_commands_queue.get_nowait()
                    commands_to_process.append(command)
                except queue.Empty:
                    break

            for command in commands_to_process:
                if self.control_loop:
                    await self.control_loop._handle_command(command)

        except Exception as e:
            logger.error(f"Error processing control commands: {e}")

    def restart(self):
        """Restart the teleoperation system."""

        def do_restart():
            try:
                logger.info("Initiating system restart...")
                if self.main_loop and not self.main_loop.is_closed():
                    future = asyncio.run_coroutine_threadsafe(self._soft_restart_sequence(), self.main_loop)
                    future.result(timeout=30.0)
                else:
                    logger.error("Main event loop not available for restart")
            except Exception as e:
                logger.error(f"Error during restart: {e}")

        restart_thread = threading.Thread(target=do_restart, daemon=True)
        restart_thread.start()

    async def _soft_restart_sequence(self):
        """Perform a soft restart by reinitializing components without exiting the process."""
        try:
            logger.info("Starting soft restart sequence...")
            await asyncio.sleep(1)

            for task in self.tasks:
                task.cancel()

            if self.tasks:
                try:
                    await asyncio.wait_for(
                        asyncio.gather(*self.tasks, return_exceptions=True),
                        timeout=5.0
                    )
                except asyncio.TimeoutError:
                    logger.warning("Some tasks did not complete within timeout")

            await self.control_loop.stop()
            await self.web_keyboard_handler.stop()
            await self.vr_server.stop()

            await asyncio.sleep(1)

            from .config import get_config_data
            file_config = get_config_data()
            logger.info("Configuration reloaded from file")

            self.command_queue = asyncio.Queue()
            self.control_commands_queue = queue.Queue(maxsize=10)

            self.vr_server = VRWebSocketServer(self.command_queue, self.config)
            self.web_keyboard_handler = WebKeyboardHandler(self.command_queue, self.config)
            self.control_loop = ControlLoop(self.command_queue, self.config, self.control_commands_queue)

            self.control_loop.web_keyboard_handler = self.web_keyboard_handler
            self.web_keyboard_handler.disconnect_callback = lambda: self.add_control_command("robot_disconnect")

            self.tasks = []

            await self.vr_server.start()
            await self.web_keyboard_handler.start()

            control_task = asyncio.create_task(self.control_loop.start())
            self.tasks.append(control_task)

            command_processor_task = asyncio.create_task(self._run_command_processor())
            self.tasks.append(command_processor_task)

            logger.info("System restart completed successfully")

            if self.config.autoconnect and self.config.enable_robot:
                logger.info("🔌 Auto-connecting to robot motors after restart...")
                await asyncio.sleep(0.5)
                self.add_control_command("robot_connect")

        except Exception as e:
            logger.error(f"Error during soft restart sequence: {e}")
            raise

    async def start(self):
        """Start all system components."""
        try:
            self.is_running = True
            self.main_loop = asyncio.get_event_loop()

            await self.https_server.start()
            await self.vr_server.start()
            await self.web_keyboard_handler.start()

            control_task = asyncio.create_task(self.control_loop.start())
            self.tasks.append(control_task)

            command_processor_task = asyncio.create_task(self._run_command_processor())
            self.tasks.append(command_processor_task)

            logger.info("All system components started successfully")

            if self.config.autoconnect and self.config.enable_robot:
                logger.info("🔌 Auto-connecting to robot motors...")
                await asyncio.sleep(0.5)
                self.add_control_command("robot_connect")

            while self.is_running:
                try:
                    await asyncio.gather(*self.tasks)
                    break
                except asyncio.CancelledError:
                    if self.is_running:
                        await asyncio.sleep(1)
                        continue
                    break
                except Exception as e:
                    logger.error(f"Error in main task loop: {e}")
                    break

        except OSError as e:
            if e.errno == 98:
                logger.error(f"Error starting teleoperation system: {e}")
                logger.error("To find and kill the process using these ports, run:")
                logger.error(f"  kill -9 $(lsof -t -i:{self.config.https_port} -i:{self.config.websocket_port})")
            else:
                logger.error(f"Error starting teleoperation system: {e}")
            await self.stop()
            raise
        except Exception as e:
            logger.error(f"Error starting teleoperation system: {e}")
            await self.stop()
            raise

    async def _run_command_processor(self):
        """Run the control command processor loop."""
        while self.is_running:
            await self.process_control_commands()
            await asyncio.sleep(0.05)

    async def stop(self):
        """Stop all system components."""
        logger.info("Shutting down teleoperation system...")
        self.is_running = False

        try:
            await asyncio.wait_for(self.vr_server.stop(), timeout=2.0)
        except asyncio.TimeoutError:
            logger.warning("VR server stop timed out")
        except Exception as e:
            logger.warning(f"Error stopping VR server: {e}")

        for task in self.tasks:
            task.cancel()

        if self.tasks:
            try:
                await asyncio.wait_for(
                    asyncio.gather(*self.tasks, return_exceptions=True),
                    timeout=2.0
                )
            except asyncio.TimeoutError:
                logger.warning("Some tasks did not complete within timeout")

        try:
            await asyncio.wait_for(self.control_loop.stop(), timeout=3.0)
        except asyncio.TimeoutError:
            logger.warning("Control loop stop timed out")
        except Exception as e:
            logger.warning(f"Error stopping control loop: {e}")

        try:
            await asyncio.wait_for(self.web_keyboard_handler.stop(), timeout=1.0)
        except asyncio.TimeoutError:
            logger.warning("Web keyboard handler stop timed out")
        except Exception as e:
            logger.warning(f"Error stopping web keyboard handler: {e}")

        try:
            await asyncio.wait_for(self.https_server.stop(), timeout=2.0)
        except asyncio.TimeoutError:
            logger.warning("HTTPS server stop timed out")
        except Exception as e:
            logger.warning(f"Error stopping HTTPS server: {e}")

        logger.info("Teleoperation system shutdown complete")


def create_signal_handler(system: TelegripSystem, loop: asyncio.AbstractEventLoop):
    """Create a signal handler that properly stops the system."""

    def signal_handler(signum, frame):
        logger.info(f"Received signal {signum}")
        system.is_running = False
        for task in system.tasks:
            loop.call_soon_threadsafe(task.cancel)
        raise SystemExit(0)

    return signal_handler
