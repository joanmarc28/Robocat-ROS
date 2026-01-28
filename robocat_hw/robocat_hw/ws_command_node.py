import asyncio
import json
import os
from pathlib import Path
import sys
import threading
from typing import Optional
from urllib.parse import urlparse, urlencode, parse_qsl

import rclpy
from rclpy.node import Node
from robocat_msgs.srv import SetLanguage

# Allow running with system python while loading venv packages.
_venv_path = os.environ.get("VIRTUAL_ENV")
if _venv_path:
    _site = Path(_venv_path) / "lib" / f"python{sys.version_info.major}.{sys.version_info.minor}" / "site-packages"
    if _site.exists() and str(_site) not in sys.path:
        sys.path.insert(0, str(_site))

try:
    import websockets
except Exception as exc:  # pragma: no cover
    websockets = None  # type: ignore[assignment]
    _WS_IMPORT_ERROR = exc
else:
    _WS_IMPORT_ERROR = None


class WebsocketCommandNode(Node):
    def __init__(self) -> None:
        super().__init__("ws_command_node")
        self.declare_parameter("enabled", True)
        self.declare_parameter("api_base_url", "https://europerobotics.jmprojects.cat")
        self.declare_parameter("ws_path", "/ws/telemetria")
        self.declare_parameter("access_token_path", "/home/robocat-v2/.robocat/access_token.json")
        self.declare_parameter("reconnect_sec", 2.0)
        self.declare_parameter("audio_service_name", "/audio/set_language")
        self.declare_parameter("audio_service_timeout_sec", 5.0)

        self._stop = asyncio.Event()
        self._token_cache: Optional[str] = None
        self._token_mtime: Optional[float] = None

        if _WS_IMPORT_ERROR is not None:
            self.get_logger().error("websockets not installed. Install with: sudo apt install python3-websockets")
            return

        self._thread = threading.Thread(target=self._run_thread, daemon=True)
        self._thread.start()

    def destroy_node(self) -> bool:
        self._stop.set()
        if hasattr(self, "_thread") and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        return super().destroy_node()

    def _run_thread(self) -> None:
        asyncio.run(self._run())

    def _load_access_token(self) -> Optional[str]:
        path = Path(self.get_parameter("access_token_path").value)
        if not path.exists():
            self._token_cache = None
            self._token_mtime = None
            return None
        stat = path.stat()
        if self._token_mtime == stat.st_mtime and self._token_cache:
            return self._token_cache
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            self._token_cache = None
            self._token_mtime = None
            return None
        token = data.get("access_token")
        if not token:
            self._token_cache = None
            self._token_mtime = None
            return None
        self._token_cache = token
        self._token_mtime = stat.st_mtime
        return token

    def _build_ws_url(self) -> str:
        base = str(self.get_parameter("api_base_url").value).strip().rstrip("/")
        ws_path = str(self.get_parameter("ws_path").value).strip()
        if not ws_path.startswith("/"):
            ws_path = "/" + ws_path
        parsed = urlparse(base)
        scheme = "wss" if parsed.scheme == "https" else "ws"
        host = parsed.netloc or parsed.path
        return f"{scheme}://{host}{ws_path}"

    def _append_token(self, ws_url: str, token: str) -> str:
        parsed = urlparse(ws_url)
        query = dict(parse_qsl(parsed.query))
        query["robot_token"] = token
        new_query = urlencode(query)
        return parsed._replace(query=new_query).geturl()

    def _call_set_language(self, language: str) -> tuple[bool, str]:
        service_name = str(self.get_parameter("audio_service_name").value).strip()
        timeout_sec = float(self.get_parameter("audio_service_timeout_sec").value)

        client = self.create_client(SetLanguage, service_name)
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return False, "robot not connected"
        req = SetLanguage.Request()
        req.language = language
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            return False, "no s'ha pogut aplicar"
        if future.exception():
            return False, str(future.exception())
        response = future.result()
        return bool(response.success), response.message or ""

    async def _handle_message(self, websocket, message: dict) -> None:
        msg_type = str(message.get("type") or "").strip().lower()
        request_id = message.get("request_id")
        if msg_type == "audio_language":
            language = str(message.get("language") or "").strip().lower()
            success = False
            response_message = ""
            if not language:
                response_message = "language is empty"
            else:
                success, response_message = await asyncio.to_thread(self._call_set_language, language)
            await websocket.send(
                json.dumps(
                    {
                        "response_to": request_id,
                        "success": success,
                        "message": response_message,
                    }
                )
            )

    async def _run(self) -> None:
        if not bool(self.get_parameter("enabled").value):
            self.get_logger().info("WS command node disabled (enabled=false).")
            return
        reconnect_sec = float(self.get_parameter("reconnect_sec").value)
        ws_url = self._build_ws_url()
        while not self._stop.is_set():
            token = self._load_access_token()
            if not token:
                await asyncio.sleep(reconnect_sec)
                continue
            ws_url_with_token = self._append_token(ws_url, token)
            try:
                async with websockets.connect(ws_url_with_token) as websocket:
                    self.get_logger().info("WS command connected.")
                    while not self._stop.is_set():
                        raw = await websocket.recv()
                        try:
                            message = json.loads(raw)
                        except json.JSONDecodeError:
                            continue
                        if isinstance(message, dict):
                            await self._handle_message(websocket, message)
            except Exception as exc:
                self.get_logger().warning(f"WS command error: {exc}")
                await asyncio.sleep(reconnect_sec)


def main() -> None:
    rclpy.init()
    node = WebsocketCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
