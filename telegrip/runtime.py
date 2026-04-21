"""
Runtime orchestration for telegrip system lifecycle.
"""

import asyncio
import logging
import threading
import subprocess
from typing import Optional

from .config import TelegripConfig
from .control_loop import ControlLoop
from .http_api import HTTPSServer
from .inputs.vr_ws_server import VRWebSocketServer

logger = logging.getLogger(__name__)


class TelegripSystem:
    """Main teleoperation system that coordinates all components."""

    def __init__(self, config: TelegripConfig):
        self.config = config

        self.command_queue = asyncio.Queue()

        self.https_server = HTTPSServer(config)
        self.vr_server = VRWebSocketServer(self.command_queue, config)
        self.control_loop = ControlLoop(self.command_queue, config)

        self.https_server.set_system_ref(self)

        self.tasks = []
        self.is_running = False
        self.main_loop = None
        self.gnirehtet_process: Optional[subprocess.Popen] = None

    async def _cancel_tasks(self, timeout_s: float):
        """取消当前后台任务并等待退出。"""
        for task in self.tasks:
            task.cancel()

        if not self.tasks:
            return

        try:
            await asyncio.wait_for(
                asyncio.gather(*self.tasks, return_exceptions=True),
                timeout=timeout_s,
            )
        except asyncio.TimeoutError:
            logger.warning("Some tasks did not complete within timeout")
        finally:
            self.tasks = []

    async def _stop_runtime_components(
        self,
        *,
        stop_control_loop: bool,
        stop_vr_server: bool,
        stop_https_server: bool,
    ):
        """按需停止运行时组件，供 stop/restart 复用。"""
        if stop_vr_server:
            try:
                await asyncio.wait_for(self.vr_server.stop(), timeout=2.0)
            except asyncio.TimeoutError:
                logger.warning("VR server stop timed out")
            except Exception as e:
                logger.warning(f"Error stopping VR server: {e}")

        await self._cancel_tasks(timeout_s=5.0)

        if stop_control_loop:
            try:
                await asyncio.wait_for(self.control_loop.stop(), timeout=3.0)
            except asyncio.TimeoutError:
                logger.warning("Control loop stop timed out")
            except Exception as e:
                logger.warning(f"Error stopping control loop: {e}")

        if stop_https_server:
            try:
                await asyncio.wait_for(self.https_server.stop(), timeout=2.0)
            except asyncio.TimeoutError:
                logger.warning("HTTPS server stop timed out")
            except Exception as e:
                logger.warning(f"Error stopping HTTPS server: {e}")

    def _recreate_runtime_components(self):
        """重建依赖 command_queue 的运行时组件。"""
        self.command_queue = asyncio.Queue()
        self.vr_server = VRWebSocketServer(self.command_queue, self.config)
        self.control_loop = ControlLoop(self.command_queue, self.config)

    async def _start_runtime_components(self, *, start_https_server: bool):
        """启动运行时组件，供 start/restart 复用。"""
        if start_https_server:
            await self.https_server.start()

        await self.vr_server.start()
        self.tasks.append(asyncio.create_task(self.control_loop.start()))

    def _start_gnirehtet(self):
        """Start gnirehtet reverse tethering process if enabled."""
        if not self.config.gnirehtet_enabled:
            return

        if self.gnirehtet_process and self.gnirehtet_process.poll() is None:
            logger.info("gnirehtet is already running")
            return

        try:
            from .utils import get_absolute_path, get_project_root

            gnirehtet_path = get_absolute_path(self.config.gnirehtet_binary)
            if not gnirehtet_path.exists():
                logger.warning(f"gnirehtet binary not found: {gnirehtet_path}")
                return

            cmd = [str(gnirehtet_path), self.config.gnirehtet_mode] + (self.config.gnirehtet_args or [])
            self.gnirehtet_process = subprocess.Popen(
                cmd,
                cwd=str(get_project_root()),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
            )
            logger.info(f"Started gnirehtet: {' '.join(cmd)} (pid={self.gnirehtet_process.pid})")
        except Exception as e:
            logger.error(f"Failed to start gnirehtet: {e}")

    def _stop_gnirehtet(self):
        """Stop gnirehtet process if running."""
        if not self.gnirehtet_process:
            return

        if self.gnirehtet_process.poll() is not None:
            self.gnirehtet_process = None
            return

        try:
            self.gnirehtet_process.terminate()
            self.gnirehtet_process.wait(timeout=3)
            logger.info("Stopped gnirehtet process")
        except subprocess.TimeoutExpired:
            self.gnirehtet_process.kill()
            logger.warning("gnirehtet did not stop in time, process killed")
        except Exception as e:
            logger.warning(f"Failed to stop gnirehtet process cleanly: {e}")
        finally:
            self.gnirehtet_process = None

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
            await self._stop_runtime_components(
                stop_control_loop=True,
                stop_vr_server=True,
                stop_https_server=False,
            )

            await asyncio.sleep(1)

            logger.info("Configuration reloaded from file")
            self._recreate_runtime_components()
            await self._start_runtime_components(start_https_server=False)

            logger.info("System restart completed successfully")

            if self.config.autoconnect and self.config.enable_robot:
                logger.info("🔌 Auto-connect command skipped: robot backend now reconnects automatically")

        except Exception as e:
            logger.error(f"Error during soft restart sequence: {e}")
            raise

    async def start(self):
        """Start all system components."""
        try:
            self.is_running = True
            self.main_loop = asyncio.get_event_loop()
            self._start_gnirehtet()
            await self._start_runtime_components(start_https_server=True)

            logger.info("All system components started successfully")

            if self.config.autoconnect and self.config.enable_robot:
                logger.info("🔌 Auto-connect command skipped: robot backend now reconnects automatically")

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

    async def stop(self):
        """Stop all system components."""
        logger.info("Shutting down teleoperation system...")
        self.is_running = False
        self._stop_gnirehtet()
        await self._stop_runtime_components(
            stop_control_loop=True,
            stop_vr_server=True,
            stop_https_server=True,
        )

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
