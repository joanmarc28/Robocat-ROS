import base64
import json
import os
import secrets
import threading
import time
import uuid
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import requests
except ImportError:
    requests = None


@dataclass
class Identity:
    robot_id: str
    robot_type: str
    serial_number: Optional[str]
    firmware_version: Optional[str]
    device_secret: Optional[str]
    paired_at: Optional[str]


class PairingNode(Node):
    def __init__(self) -> None:
        super().__init__("pairing_node")
        self.declare_parameter("api_base_url", "")
        self.declare_parameter("app_base_url", "")
        self.declare_parameter("pair_start_path", "/api/robots/pair/start")
        self.declare_parameter("pair_status_path", "/api/robots/pair/status")
        self.declare_parameter("auth_path", "/api/robots/auth")
        self.declare_parameter("robot_type", "NOVA")
        self.declare_parameter("serial_number", "")
        self.declare_parameter("firmware_version", "")
        self.declare_parameter("identity_path", "/var/lib/robocat/identity.json")
        self.declare_parameter("access_token_path", "/var/lib/robocat/access_token.json")
        self.declare_parameter("token_ttl_sec", 300)
        self.declare_parameter("retry_sec", 2.0)
        self.declare_parameter("show_qr", True)
        self.declare_parameter("oled_text_topic", "oled_text")
        self.declare_parameter("oled_qr_topic", "oled_qr")

        self._state_pub = self.create_publisher(String, "/robot_pairing/state", 10)
        self._oled_text_pub = self.create_publisher(
            String, self.get_parameter("oled_text_topic").value, 10
        )
        self._oled_qr_pub = self.create_publisher(
            String, self.get_parameter("oled_qr_topic").value, 10
        )
        self._stop = threading.Event()

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _publish_state(self, value: str) -> None:
        msg = String()
        msg.data = value
        self._state_pub.publish(msg)

    def _identity_path(self) -> Path:
        return Path(self.get_parameter("identity_path").value)

    def _access_token_path(self) -> Path:
        return Path(self.get_parameter("access_token_path").value)

    def _load_identity(self) -> Optional[Identity]:
        path = self._identity_path()
        if not path.exists():
            return None
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return None
        robot_id = data.get("robot_id")
        if not robot_id:
            return None
        return Identity(
            robot_id=robot_id,
            robot_type=data.get("robot_type") or self.get_parameter("robot_type").value,
            serial_number=data.get("serial_number") or None,
            firmware_version=data.get("firmware_version") or None,
            device_secret=data.get("device_secret") or None,
            paired_at=data.get("paired_at") or None,
        )

    def _write_identity(self, identity: Identity) -> None:
        path = self._identity_path()
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "robot_id": identity.robot_id,
            "device_secret": identity.device_secret,
            "paired_at": identity.paired_at,
            "robot_type": identity.robot_type,
            "serial_number": identity.serial_number,
            "firmware_version": identity.firmware_version,
        }
        tmp_path = path.with_suffix(path.suffix + ".tmp")
        tmp_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        os.replace(tmp_path, path)
        try:
            os.chmod(path, 0o600)
        except OSError:
            pass

    def _write_access_token(self, token: str, expires_at: Optional[datetime]) -> None:
        path = self._access_token_path()
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "access_token": token,
            "expires_at": expires_at.isoformat() if expires_at else None,
        }
        tmp_path = path.with_suffix(path.suffix + ".tmp")
        tmp_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        os.replace(tmp_path, path)
        try:
            os.chmod(path, 0o600)
        except OSError:
            pass

    def _clear_access_token(self) -> None:
        path = self._access_token_path()
        try:
            path.unlink()
        except OSError:
            pass

    def _make_robot_id(self) -> str:
        return str(uuid.uuid4())

    def _generate_token(self, length: int = 8) -> str:
        raw = base64.b32encode(secrets.token_bytes(5)).decode("ascii").rstrip("=")
        return raw[:length].upper()

    def _publish_oled_text(self, lines: list[str]) -> None:
        msg = String()
        msg.data = "\n".join(lines)
        self._oled_text_pub.publish(msg)

    def _publish_oled_qr(self, url: str) -> None:
        msg = String()
        msg.data = url
        self._oled_qr_pub.publish(msg)

    def _build_url(self, base: str, path: str) -> str:
        base = base.rstrip("/")
        path = path if path.startswith("/") else f"/{path}"
        return f"{base}{path}"

    def _pair_start(self, identity: Identity, token: str, expires_at: datetime) -> bool:
        if requests is None:
            self.get_logger().error("python3-requests is not installed.")
            return False
        api_base = self.get_parameter("api_base_url").value
        if not api_base:
            self.get_logger().error("api_base_url is empty.")
            return False
        url = self._build_url(api_base, self.get_parameter("pair_start_path").value)
        payload = {
            "robot_id": identity.robot_id,
            "robot_type": identity.robot_type,
            "serial_number": identity.serial_number,
            "firmware_version": identity.firmware_version,
            "token": token,
            "expires_at": expires_at.isoformat(),
        }
        try:
            response = requests.post(url, json=payload, timeout=5)
            if response.status_code >= 400:
                self.get_logger().warning(f"Pair start failed: {response.status_code}")
                return False
            return True
        except requests.RequestException as exc:
            self.get_logger().warning(f"Pair start error: {exc}")
            return False

    def _pair_status(self, token: str) -> Optional[Dict[str, Any]]:
        if requests is None:
            return None
        api_base = self.get_parameter("api_base_url").value
        if not api_base:
            return None
        url = self._build_url(api_base, self.get_parameter("pair_status_path").value)
        try:
            response = requests.get(url, params={"token": token}, timeout=5)
            if response.status_code == 404:
                return {"status": "INVALID"}
            if response.status_code >= 400:
                self.get_logger().warning(f"Pair status error: {response.status_code}")
                return None
            return response.json()
        except requests.RequestException:
            return None
        except ValueError:
            return None

    def _auth_robot(self, identity: Identity) -> Optional[Dict[str, Any]]:
        if requests is None:
            return None
        api_base = self.get_parameter("api_base_url").value
        if not api_base:
            return None
        url = self._build_url(api_base, self.get_parameter("auth_path").value)
        payload = {"robot_id": identity.robot_id, "device_secret": identity.device_secret}
        try:
            response = requests.post(url, json=payload, timeout=5)
            if response.status_code in (401, 403):
                return {"revoked": True}
            if response.status_code >= 400:
                self.get_logger().warning(f"Auth error: {response.status_code}")
                return None
            return response.json()
        except requests.RequestException:
            return None
        except ValueError:
            return None

    def _ensure_identity(self) -> Identity:
        identity = self._load_identity()
        if identity is None:
            identity = Identity(
                robot_id=self._make_robot_id(),
                robot_type=self.get_parameter("robot_type").value,
                serial_number=self.get_parameter("serial_number").value or None,
                firmware_version=self.get_parameter("firmware_version").value or None,
                device_secret=None,
                paired_at=None,
            )
            self._write_identity(identity)
        return identity

    def _pairing_loop(self, identity: Identity) -> Identity:
        token_ttl = int(self.get_parameter("token_ttl_sec").value)
        retry_sec = float(self.get_parameter("retry_sec").value)
        app_base = self.get_parameter("app_base_url").value.rstrip("/")
        show_qr = bool(self.get_parameter("show_qr").value)

        while not self._stop.is_set() and not identity.device_secret:
            token = self._generate_token()
            expires_at = datetime.now(timezone.utc) + timedelta(seconds=token_ttl)

            self._publish_state("STARTING")
            while not self._stop.is_set():
                started = self._pair_start(identity, token, expires_at)
                if started:
                    break
                self._publish_state("ERROR")
                self._publish_oled_text(["Connectant...", "Sense connexio"])
                time.sleep(retry_sec)

            if self._stop.is_set():
                break

            self._publish_state("WAITING_CONFIRM")
            pair_url = f"{app_base}/pair?token={token}" if app_base else f"/pair?token={token}"
            if show_qr:
                self._publish_oled_qr(pair_url)
            else:
                self._publish_oled_text(["URL:", app_base or "-", f"Token: {token}"])

            while not self._stop.is_set():
                if datetime.now(timezone.utc) >= expires_at:
                    self._publish_oled_text(["Token expirat", "Generant nou"])
                    break
                status = self._pair_status(token)
                if status is None:
                    self._publish_oled_text(["Sense connexio", "Reintentant"])
                    time.sleep(retry_sec)
                    continue
                state = (status.get("status") or "").upper()
                if state in ("CONFIRMED",) or status.get("device_secret"):
                    device_secret = status.get("device_secret")
                    if not device_secret:
                        self.get_logger().warning("Confirmed without device_secret.")
                        time.sleep(retry_sec)
                        continue
                    identity.device_secret = device_secret
                    identity.paired_at = datetime.now(timezone.utc).isoformat()
                    self._write_identity(identity)
                    self._publish_oled_text(["Vinculat OK"])
                    time.sleep(3.0)
                    self._publish_state("PAIRED")
                    return identity
                if state in ("EXPIRED", "CANCELLED", "INVALID"):
                    self._publish_oled_text(["Token invalid", "Reintentant"])
                    break
                time.sleep(retry_sec)

        return identity

    def _auth_loop(self, identity: Identity) -> Optional[Identity]:
        retry_sec = float(self.get_parameter("retry_sec").value)
        while not self._stop.is_set() and identity.device_secret:
            auth = self._auth_robot(identity)
            if auth is None:
                time.sleep(retry_sec)
                continue
            if auth.get("revoked"):
                self.get_logger().error("Device secret revoked, returning to pairing.")
                identity.device_secret = None
                identity.paired_at = None
                self._write_identity(identity)
                self._clear_access_token()
                self._publish_state("UNPAIRED")
                return identity
            token = auth.get("access_token")
            if not token:
                time.sleep(retry_sec)
                continue
            expires_in = auth.get("expires_in")
            expires_at = None
            expires_at_raw = auth.get("expires_at")
            if isinstance(expires_in, (int, float)):
                expires_at = datetime.now(timezone.utc) + timedelta(seconds=int(expires_in))
            elif isinstance(expires_at_raw, str) and expires_at_raw:
                try:
                    expires_at = datetime.fromisoformat(expires_at_raw.replace("Z", "+00:00"))
                except ValueError:
                    expires_at = None
            self._write_access_token(token, expires_at)
            self._publish_state("PAIRED")
            time.sleep(max(10.0, retry_sec))
        return identity

    def _run(self) -> None:
        self._publish_state("UNPAIRED")
        while not self._stop.is_set():
            identity = self._ensure_identity()
            if identity.device_secret:
                self._publish_state("PAIRED")
                identity = self._auth_loop(identity) or identity
            else:
                self._publish_state("UNPAIRED")
                identity = self._pairing_loop(identity)
            time.sleep(0.5)


def main() -> None:
    rclpy.init()
    node = PairingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop.set()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
