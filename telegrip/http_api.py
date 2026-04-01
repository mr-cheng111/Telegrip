"""
HTTP API server components for telegrip.
"""

import http.server
import json
import logging
import socket
import ssl
import threading

from .config import TelegripConfig, get_config_data, update_config_data
from .utils import get_absolute_path

logger = logging.getLogger(__name__)


def get_local_ip():
    """Get the local IP address of this machine."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except Exception:
        try:
            return socket.gethostbyname(socket.gethostname())
        except Exception:
            return "localhost"


class APIHandler(http.server.BaseHTTPRequestHandler):
    """HTTP request handler for the teleoperation API."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        try:
            super().end_headers()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, ssl.SSLError):
            pass

    def do_OPTIONS(self):
        self.send_response(200)
        self.end_headers()

    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if self.path == '/api/status':
            self.handle_status_request()
        elif self.path == '/api/config':
            self.handle_config_get_request()
        else:
            if not self._serve_static_file():
                self.send_error(404, "Not found")

    def do_POST(self):
        if self.path == '/api/keyboard':
            self.handle_keyboard_request()
        elif self.path == '/api/robot':
            self.handle_robot_request()
        elif self.path == '/api/keypress':
            self.handle_keypress_request()
        elif self.path == '/api/config':
            self.handle_config_post_request()
        elif self.path == '/api/restart':
            self.handle_restart_request()
        else:
            self.send_error(404, "Not found")

    def handle_status_request(self):
        try:
            system = self._get_system()
            if not system:
                self.send_error(500, "System not available")
                return

            control_status = system.control_loop.status if system.control_loop else {}

            robot_engaged = False
            if system.control_loop and system.control_loop.robot_interface:
                robot_engaged = system.control_loop.robot_interface.is_engaged

            vr_connected = False
            if system.vr_server and system.vr_server.is_running:
                vr_connected = len(system.vr_server.clients) > 0

            status = {
                **control_status,
                "keyboardEnabled": False,
                "robotEngaged": robot_engaged,
                "vrConnected": vr_connected
            }
            self._send_json_response(status)

        except Exception as e:
            logger.error(f"Error handling status request: {e}")
            self.send_error(500, str(e))

    def handle_keyboard_request(self):
        self._send_json_response(
            {
                "success": False,
                "error": "Keyboard control has been removed"
            },
            status=410,
        )

    def handle_robot_request(self):
        try:
            data = self._read_json_body()
            action = data.get('action')
            logger.info(f"🔌 Received robot action: {action}")

            if action in ['connect', 'disconnect']:
                system = self._get_system()
                if not system:
                    logger.error("🔌 Server api_handler not available")
                    self.send_error(500, "System not available")
                    return
                command_name = f"robot_{action}"
                logger.info(f"🔌 Adding command to queue: {command_name}")
                system.add_control_command(command_name)
                self._send_json_response({"success": True, "action": action})
            else:
                self.send_error(400, f"Invalid action: {action}")

        except ValueError as e:
            self.send_error(400, str(e))
        except json.JSONDecodeError:
            self.send_error(400, "Invalid JSON")
        except Exception as e:
            logger.error(f"Error handling robot request: {e}")
            self.send_error(500, str(e))

    def handle_keypress_request(self):
        self._send_json_response(
            {
                "success": False,
                "error": "Keyboard control has been removed"
            },
            status=410,
        )

    def handle_config_get_request(self):
        try:
            config_data = get_config_data()
            self._send_json_response(config_data)

        except Exception as e:
            logger.error(f"Error handling config get request: {e}")
            self.send_error(500, str(e))

    def handle_config_post_request(self):
        try:
            data = self._read_json_body()
            success = update_config_data(data)

            if success:
                self._send_json_response({"success": True, "message": "Configuration updated successfully"})
                logger.info("Configuration updated successfully")
            else:
                self.send_error(500, "Failed to save configuration")

        except ValueError as e:
            self.send_error(400, str(e))
        except json.JSONDecodeError:
            self.send_error(400, "Invalid JSON")
        except Exception as e:
            logger.error(f"Error handling config post request: {e}")
            self.send_error(500, str(e))

    def handle_restart_request(self):
        try:
            system = self._get_system()
            if not system:
                self.send_error(500, "System not available")
                return
            logger.info("Restarting teleoperation system...")
            system.restart()
            self._send_json_response({"success": True, "message": "Teleoperation system restarted"})

        except Exception as e:
            logger.error(f"Error handling restart request: {e}")
            self.send_error(500, str(e))

    def _get_system(self):
        if hasattr(self.server, 'api_handler') and self.server.api_handler:
            return self.server.api_handler
        return None

    def _read_json_body(self):
        content_length = int(self.headers.get('Content-Length', 0))
        if content_length == 0:
            raise ValueError("No request body")
        post_data = self.rfile.read(content_length)
        return json.loads(post_data.decode('utf-8'))

    def _send_json_response(self, data, status: int = 200):
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode('utf-8'))

    def _serve_static_file(self) -> bool:
        if self.path in ('/', '/index.html'):
            self.serve_file('web-ui/index.html', 'text/html')
            return True

        static_types = {
            '.css': ('text/css', True),
            '.js': ('application/javascript', True),
            '.jpg': ('image/jpeg', True),
            '.jpeg': ('image/jpeg', True),
            '.png': ('image/png', True),
            '.gif': ('image/gif', True),
            '.ico': ('image/x-icon', False),
        }
        for suffix, (content_type, under_web_ui) in static_types.items():
            if self.path.endswith(suffix):
                filename = f'web-ui{self.path}' if under_web_ui else self.path[1:]
                self.serve_file(filename, content_type)
                return True
        return False

    def serve_file(self, filename, content_type):
        try:
            abs_path = get_absolute_path(filename)
            with open(abs_path, 'rb') as f:
                file_content = f.read()

            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', len(file_content))
            self.end_headers()
            self.wfile.write(file_content)

        except FileNotFoundError:
            self.send_error(404, f"File {filename} not found")
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            logger.debug(f"Client disconnected while serving {filename}")
        except Exception as e:
            logger.error(f"Error serving file {filename}: {e}")
            try:
                self.send_error(500, "Internal server error")
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass


class HTTPSServer:
    """HTTPS server for the teleoperation API."""

    def __init__(self, config: TelegripConfig):
        self.config = config
        self.httpd = None
        self.server_thread = None
        self.system_ref = None

    def set_system_ref(self, system_ref):
        self.system_ref = system_ref

    async def start(self):
        try:
            self.httpd = http.server.HTTPServer((self.config.host_ip, self.config.https_port), APIHandler)
            self.httpd.api_handler = self.system_ref

            context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            cert_path, key_path = self.config.get_absolute_ssl_paths()
            context.load_cert_chain(cert_path, key_path)
            self.httpd.socket = context.wrap_socket(self.httpd.socket, server_side=True)

            self.server_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
            self.server_thread.start()

            if getattr(logging, self.config.log_level.upper()) <= logging.INFO:
                host_display = get_local_ip() if self.config.host_ip == "0.0.0.0" else self.config.host_ip
                logger.info(f"HTTPS server started on https://{host_display}:{self.config.https_port}")

        except Exception as e:
            logger.error(f"Failed to start HTTPS server: {e}")
            raise

    async def stop(self):
        if self.httpd:
            self.httpd.shutdown()
            if self.server_thread:
                self.server_thread.join(timeout=5)
            logger.info("HTTPS server stopped")
