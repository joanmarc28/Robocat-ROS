import asyncio
import json
import os
from pathlib import Path
import sys
import threading
from typing import Any, Dict, Optional

# Allow running with system python while loading venv packages (aiortc/av).
_venv_path = os.environ.get("VIRTUAL_ENV")
if _venv_path:
    _site = Path(_venv_path) / "lib" / f"python{sys.version_info.major}.{sys.version_info.minor}" / "site-packages"
    if _site.exists() and str(_site) not in sys.path:
        sys.path.insert(0, str(_site))

import rclpy
from rclpy.node import Node

try:
    from aiortc import RTCIceCandidate, RTCPeerConnection, RTCSessionDescription
    from aiortc.contrib.media import MediaPlayer
    _AIORTC_IMPORT_ERROR: Optional[Exception] = None
except Exception as exc:  # pragma: no cover - optional runtime dependency
    RTCPeerConnection = None  # type: ignore[assignment]
    RTCSessionDescription = None  # type: ignore[assignment]
    RTCIceCandidate = None  # type: ignore[assignment]
    MediaPlayer = None  # type: ignore[assignment]
    _AIORTC_IMPORT_ERROR = exc

try:
    import requests
except Exception as exc:  # pragma: no cover
    requests = None  # type: ignore[assignment]
    _REQUESTS_IMPORT_ERROR = exc
else:
    _REQUESTS_IMPORT_ERROR = None


class WebRtcSignalingNode(Node):
    def __init__(self) -> None:
        super().__init__("webrtc_signaling_node")
        self.declare_parameter("enabled", True)
        self.declare_parameter("api_base_url", "https://europerobotics.jmprojects.cat")
        self.declare_parameter("video_base_path", "/api/video")
        self.declare_parameter("video_offer_path", "/offer")
        self.declare_parameter("video_answer_path", "/answer")
        self.declare_parameter("offer_query", "")
        self.declare_parameter("ice_query", "role=robot")
        self.declare_parameter("include_robot_id", True)
        self.declare_parameter("include_stream_token", True)
        self.declare_parameter("debug_signaling", False)
        self.declare_parameter("identity_path", "/home/robocat-v2/.robocat/identity.json")
        self.declare_parameter("access_token_path", "/home/robocat-v2/.robocat/access_token.json")
        self.declare_parameter("offer_poll_sec", 1.0)
        self.declare_parameter("ice_poll_sec", 1.0)
        self.declare_parameter("camera_device", "/dev/video0")
        self.declare_parameter("camera_format", "v4l2")
        self.declare_parameter("camera_pixel_format", "")
        self.declare_parameter("camera_width", 1280)
        self.declare_parameter("camera_height", 720)
        self.declare_parameter("camera_fps", 30)

        self._stop = asyncio.Event()
        self._token_cache: Optional[str] = None
        self._token_mtime: Optional[float] = None
        self._session = requests.Session() if requests else None

        if _AIORTC_IMPORT_ERROR is not None:
            self.get_logger().error(
                "aiortc not installed. Install with: pip3 install aiortc av"
            )
            return
        if _REQUESTS_IMPORT_ERROR is not None:
            self.get_logger().error(
                "requests not installed. Install with: sudo apt install python3-requests"
            )
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

    def _base_url(self) -> str:
        base = str(self.get_parameter("api_base_url").value).strip().rstrip("/")
        return base

    def _video_base_url(self) -> str:
        base = self._base_url()
        path = str(self.get_parameter("video_base_path").value).strip()
        if not path.startswith("/"):
            path = "/" + path
        return f"{base}{path}"

    def _build_url(self, base: str, query: str) -> str:
        query = query.strip()
        if not query:
            return base
        if query.startswith("?"):
            return f"{base}{query}"
        return f"{base}?{query}"

    def _load_robot_id(self) -> Optional[str]:
        if not bool(self.get_parameter("include_robot_id").value):
            return None
        path = Path(self.get_parameter("identity_path").value)
        if not path.exists():
            return None
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            return None
        value = data.get("robot_id")
        if not value:
            return None
        return str(value)

    async def _http_get_json(
        self, url: str, headers: Dict[str, str]
    ) -> tuple[Optional[Dict[str, Any]], Optional[int]]:
        if not self._session:
            return None, None
        try:
            response = await asyncio.to_thread(
                self._session.get,
                url,
                headers=headers,
                timeout=10,
            )
        except Exception as exc:
            self.get_logger().warning(f"HTTP GET failed: {exc}")
            return None, None
        if response.status_code in (204, 404):
            return None, response.status_code
        if response.status_code >= 400:
            self.get_logger().warning(f"HTTP GET {url} -> {response.status_code}")
            return None, response.status_code
        try:
            return response.json(), response.status_code
        except Exception as exc:
            self.get_logger().warning(f"Invalid JSON from {url}: {exc}")
            return None, response.status_code

    async def _http_post_json(
        self, url: str, headers: Dict[str, str], payload: Dict[str, Any]
    ) -> bool:
        if not self._session:
            return False
        try:
            response = await asyncio.to_thread(
                self._session.post,
                url,
                headers=headers,
                json=payload,
                timeout=10,
            )
        except Exception as exc:
            self.get_logger().warning(f"HTTP POST failed: {exc}")
            return False
        if response.status_code >= 400:
            self.get_logger().warning(f"HTTP POST {url} -> {response.status_code}")
            return False
        return True

    async def _http_post_json_response(
        self, url: str, headers: Dict[str, str], payload: Dict[str, Any]
    ) -> tuple[Optional[Dict[str, Any]], Optional[int]]:
        if not self._session:
            return None, None
        try:
            response = await asyncio.to_thread(
                self._session.post,
                url,
                headers=headers,
                json=payload,
                timeout=10,
            )
        except Exception as exc:
            self.get_logger().warning(f"HTTP POST failed: {exc}")
            return None, None
        if response.status_code in (204, 404):
            return None, response.status_code
        if response.status_code >= 400:
            self.get_logger().warning(f"HTTP POST {url} -> {response.status_code}")
            return None, response.status_code
        try:
            return response.json(), response.status_code
        except Exception as exc:
            self.get_logger().warning(f"Invalid JSON from {url}: {exc}")
            return None, response.status_code

    def _parse_candidates(self, payload: Any) -> list[Dict[str, Any]]:
        if payload is None:
            return []
        if isinstance(payload, list):
            return [c for c in payload if isinstance(c, dict)]
        if isinstance(payload, dict):
            if "candidates" in payload and isinstance(payload["candidates"], list):
                return [c for c in payload["candidates"] if isinstance(c, dict)]
            if "candidate" in payload:
                return [payload]
        return []

    async def _poll_remote_ice(self, pc: RTCPeerConnection, headers: Dict[str, str]) -> None:
        ice_url = f"{self._video_base_url()}/ice"
        ice_query = str(self.get_parameter("ice_query").value).strip()
        ice_url = self._build_url(ice_url, ice_query)
        poll_sec = float(self.get_parameter("ice_poll_sec").value)
        while not self._stop.is_set():
            if pc.connectionState in ("failed", "closed"):
                return
            payload, _ = await self._http_get_json(ice_url, headers)
            for cand in self._parse_candidates(payload):
                try:
                    cand_payload = cand.get("candidate", cand)
                    if isinstance(cand_payload, dict):
                        candidate = RTCIceCandidate(
                            candidate=cand_payload.get("candidate"),
                            sdpMid=cand_payload.get("sdpMid"),
                            sdpMLineIndex=cand_payload.get("sdpMLineIndex"),
                        )
                    else:
                        candidate = RTCIceCandidate(
                            candidate=cand.get("candidate"),
                            sdpMid=cand.get("sdpMid"),
                            sdpMLineIndex=cand.get("sdpMLineIndex"),
                        )
                    await pc.addIceCandidate(candidate)
                except Exception as exc:
                    self.get_logger().warning(f"Failed to add ICE candidate: {exc}")
            await asyncio.sleep(poll_sec)

    async def _run(self) -> None:
        if not bool(self.get_parameter("enabled").value):
            self.get_logger().info("WebRTC signaling disabled (enabled=false).")
            return

        offer_path = str(self.get_parameter("video_offer_path").value).strip()
        answer_path = str(self.get_parameter("video_answer_path").value).strip()
        if not offer_path.startswith("/"):
            offer_path = "/" + offer_path
        if not answer_path.startswith("/"):
            answer_path = "/" + answer_path
        offer_url = f"{self._video_base_url()}{offer_path}"
        answer_url = f"{self._video_base_url()}{answer_path}"
        offer_poll_sec = float(self.get_parameter("offer_poll_sec").value)

        while not self._stop.is_set():
            token = self._load_access_token()
            if not token:
                await asyncio.sleep(offer_poll_sec)
                continue

            headers = {"x-robot-token": token}
            offer_query = str(self.get_parameter("offer_query").value).strip()
            robot_id = self._load_robot_id()
            if robot_id and "robot_id=" not in offer_query:
                if offer_query:
                    offer_query = f"{offer_query}&robot_id={robot_id}"
                else:
                    offer_query = f"robot_id={robot_id}"
            offer_url_with_query = self._build_url(offer_url, offer_query)

            offer_payload, offer_status = await self._http_get_json(offer_url_with_query, headers)
            if offer_status == 405:
                offer_payload, _ = await self._http_post_json_response(
                    offer_url_with_query, headers, {}
                )
            if not offer_payload:
                await asyncio.sleep(offer_poll_sec)
                continue

            if "offer" in offer_payload:
                nested = offer_payload.get("offer")
                if isinstance(nested, dict):
                    if not offer_payload.get("stream_token") and nested.get("stream_token"):
                        offer_payload["stream_token"] = nested.get("stream_token")
                    offer_payload = nested
                elif nested is None:
                    await asyncio.sleep(offer_poll_sec)
                    continue

            stream_token = ""
            if bool(self.get_parameter("include_stream_token").value):
                stream_token = str(offer_payload.get("stream_token", "")).strip()

            sdp = offer_payload.get("sdp")
            sdp_type = offer_payload.get("type", "offer")
            if not sdp:
                self.get_logger().warning("Offer missing SDP. Waiting...")
                await asyncio.sleep(offer_poll_sec)
                continue

            pc = RTCPeerConnection()

            @pc.on("icecandidate")
            async def on_icecandidate(candidate) -> None:
                if candidate is None:
                    return
                payload = {
                    "direction": "robot",
                    "candidate": {
                        "candidate": candidate.to_sdp(),
                        "sdpMid": candidate.sdpMid,
                        "sdpMLineIndex": candidate.sdpMLineIndex,
                    },
                }
                if stream_token:
                    payload["stream_token"] = stream_token
                if robot_id:
                    payload["robot_id"] = robot_id
                await self._http_post_json(f"{self._video_base_url()}/ice", headers, payload)

            device = str(self.get_parameter("camera_device").value)
            camera_format = str(self.get_parameter("camera_format").value)
            pixel_format = str(self.get_parameter("camera_pixel_format").value).strip()
            width = int(self.get_parameter("camera_width").value)
            height = int(self.get_parameter("camera_height").value)
            fps = int(self.get_parameter("camera_fps").value)

            options: Dict[str, str] = {}
            if width > 0 and height > 0:
                options["video_size"] = f"{width}x{height}"
            if fps > 0:
                options["framerate"] = str(fps)
            if pixel_format:
                options["pixel_format"] = pixel_format

            try:
                player = MediaPlayer(device, format=camera_format, options=options)
            except Exception as exc:
                self.get_logger().error(f"Failed to open camera {device}: {exc}")
                await asyncio.sleep(offer_poll_sec)
                continue

            if player.video is None:
                self.get_logger().error("Camera opened but no video track available.")
                await asyncio.sleep(offer_poll_sec)
                continue

            pc.addTrack(player.video)

            try:
                await pc.setRemoteDescription(RTCSessionDescription(sdp=sdp, type=sdp_type))
                answer = await pc.createAnswer()
                await pc.setLocalDescription(answer)
                answer_payload = {
                    "sdp": pc.localDescription.sdp,
                    "type": pc.localDescription.type,
                }
                if stream_token:
                    answer_payload["stream_token"] = stream_token
                if robot_id:
                    answer_payload["robot_id"] = robot_id
                await self._http_post_json(answer_url, headers, answer_payload)
            except Exception as exc:
                if bool(self.get_parameter("debug_signaling").value):
                    sdp_preview = sdp[:120].replace("\r", "\\r").replace("\n", "\\n") if isinstance(sdp, str) else str(sdp)
                    self.get_logger().warning(
                        f"Offer debug: type={sdp_type} len={len(sdp) if isinstance(sdp, str) else 'n/a'} "
                        f"stream_token={stream_token or ''} sdp={sdp_preview}"
                    )
                self.get_logger().warning(f"WebRTC negotiation failed: {exc}")
                await pc.close()
                await asyncio.sleep(offer_poll_sec)
                continue

            ice_task = asyncio.create_task(self._poll_remote_ice(pc, headers))
            self.get_logger().info("WebRTC connected, streaming video.")
            try:
                while not self._stop.is_set():
                    if pc.connectionState in ("failed", "closed", "disconnected"):
                        break
                    await asyncio.sleep(1.0)
            finally:
                ice_task.cancel()
                await pc.close()
                await asyncio.sleep(offer_poll_sec)


def main() -> None:
    rclpy.init()
    node = WebRtcSignalingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
