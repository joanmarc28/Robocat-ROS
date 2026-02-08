import textwrap
from urllib.parse import urlparse, parse_qs
import threading
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from luma.core.interface.serial import i2c
    from luma.oled.device import ssd1306
except ImportError:
    i2c = None
    ssd1306 = None

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    Image = None
    ImageDraw = None
    ImageFont = None

try:
    import qrcode
except ImportError:
    qrcode = None

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None  # type: ignore[assignment]


class OledDisplay:
    def __init__(self, bus: int, address: int, width: int, height: int):
        if i2c is None or ssd1306 is None:
            raise RuntimeError("luma.oled is not installed")
        self._serial = i2c(port=bus, address=address)
        self._device = ssd1306(self._serial, width=width, height=height)
        self._width = width
        self._height = height

    def show_lines(self, lines: List[str], font: "ImageFont.ImageFont", line_height: int) -> None:
        if Image is None or ImageDraw is None:
            raise RuntimeError("Pillow is not installed")
        image = Image.new("1", (self._width, self._height))
        draw = ImageDraw.Draw(image)
        for idx, line in enumerate(lines):
            y = idx * line_height
            draw.text((0, y), line, font=font, fill=255)
        self._device.display(image)

    def show_image(self, image: "Image.Image") -> None:
        self._device.display(image)

    def show_qr(
        self,
        qr_image: "Image.Image",
        lines: List[str],
        font: "ImageFont.ImageFont",
        line_height: int,
    ) -> None:
        if Image is None or ImageDraw is None:
            raise RuntimeError("Pillow is not installed")
        image = Image.new("1", (self._width, self._height))
        qr_size = min(self._height, self._width // 2)
        qr_resized = qr_image.resize((qr_size, qr_size))
        image.paste(qr_resized, (0, 0))
        draw = ImageDraw.Draw(image)
        x_offset = qr_size + 4
        for idx, line in enumerate(lines):
            y = idx * line_height
            draw.text((x_offset, y), line, font=font, fill=255)
        self._device.display(image)


class OledMessageNode(Node):
    def __init__(self) -> None:
        super().__init__("oled_message_node")
        self.declare_parameter("message", "Hello Robocat!")
        self.declare_parameter("bus_left", 3)
        self.declare_parameter("bus_right", 4)
        self.declare_parameter("address", 0x3C)
        self.declare_parameter("width", 128)
        self.declare_parameter("height", 64)
        self.declare_parameter("line_height", 10)
        self.declare_parameter("font_path", "")
        self.declare_parameter("font_size", 8)
        self.declare_parameter("assets_path", "assets/eyes_img")
        self.declare_parameter("anim_delay", 0.05)
        self.declare_parameter("anim_loop", True)
        self.declare_parameter("anim_state_topic", "/oled_anim_state")

        self._left = self._make_display(self.get_parameter("bus_left").value)
        self._right = self._make_display(self.get_parameter("bus_right").value)

        if self._left is None and self._right is None:
            self.get_logger().error("No OLED displays could be initialized.")
            return

        self._font = self._load_font()
        self._frame_cache: Dict[Path, List["Image.Image"]] = {}
        self._assets_root = self._resolve_assets_root()
        self._anim_thread: Optional[threading.Thread] = None
        self._anim_stop = threading.Event()
        self._current_anim: str = ""

        self._show_text(self.get_parameter("message").value)

        self._text_sub = self.create_subscription(String, "oled_text", self._on_text, 10)
        self._qr_sub = self.create_subscription(String, "oled_qr", self._on_qr, 10)
        self._anim_sub = self.create_subscription(String, "oled_anim", self._on_anim, 10)
        self._anim_state_pub = self.create_publisher(
            String, self.get_parameter("anim_state_topic").value, 10
        )
        self.get_logger().info("OLED message node ready.")

    def _resolve_assets_root(self) -> Path:
        configured = Path(str(self.get_parameter("assets_path").value))
        candidates: List[Path] = []
        candidates.append(configured)
        if not configured.is_absolute():
            candidates.append(Path.cwd() / configured)
        if get_package_share_directory is not None:
            try:
                share = Path(get_package_share_directory("robocat_hw"))
                candidates.append(share / "assets" / "eyes_img")
            except Exception:
                pass
        # Fallback for development runs from source workspace.
        candidates.append(Path(__file__).resolve().parents[2] / "assets" / "eyes_img")

        for path in candidates:
            if path.exists():
                self.get_logger().info(f"OLED assets path: {path}")
                return path

        self.get_logger().warning(
            f"OLED assets path not found. Tried: {', '.join(str(p) for p in candidates)}"
        )
        return configured

    def _make_display(self, bus: int) -> Optional[OledDisplay]:
        try:
            return OledDisplay(
                bus=bus,
                address=int(self.get_parameter("address").value),
                width=int(self.get_parameter("width").value),
                height=int(self.get_parameter("height").value),
            )
        except Exception as exc:
            self.get_logger().warning(f"Display init failed on I2C bus {bus}: {exc}")
            return None

    def _load_font(self) -> "ImageFont.ImageFont":
        if ImageFont is None:
            raise RuntimeError("Pillow is not installed")
        font_path = self.get_parameter("font_path").value
        font_size = int(self.get_parameter("font_size").value)
        if font_path:
            try:
                return ImageFont.truetype(font_path, font_size)
            except Exception:
                self.get_logger().warning("Font load failed, using default.")
        return ImageFont.load_default()

    def _show_text(self, message: str) -> None:
        lines = self._format_message(
            message,
            self.get_parameter("width").value,
            self.get_parameter("line_height").value,
            self._font,
        )
        self._show_lines(lines)

    def _show_lines(self, lines: List[str]) -> None:
        for display in (self._left, self._right):
            if display is not None:
                display.show_lines(lines, self._font, self.get_parameter("line_height").value)

    def _format_message(
        self,
        message: str,
        width_px: int,
        line_height: int,
        font: "ImageFont.ImageFont",
    ) -> List[str]:
        max_lines = max(1, int(self.get_parameter("height").value) // line_height)
        if "\n" in message:
            lines = [line.strip() for line in message.splitlines() if line.strip()]
        else:
            char_width = max(6, font.getbbox("M")[2] - font.getbbox("M")[0])
            max_chars = max(1, width_px // char_width)
            lines = textwrap.wrap(message, width=max_chars)
        return lines[:max_lines]

    def _on_text(self, msg: String) -> None:
        self._stop_anim()
        self._show_text(msg.data)

    def _on_qr(self, msg: String) -> None:
        url = msg.data.strip()
        if not url:
            return
        self._stop_anim()
        self._show_qr(url)

    def _show_qr(self, url: str) -> None:
        token = self._extract_token(url)
        if Image is None or ImageDraw is None or qrcode is None:
            if token:
                self._show_text(f"Token: {token}")
            else:
                self._show_text("QR no disponible")
            return
        qr = qrcode.QRCode(
            error_correction=qrcode.constants.ERROR_CORRECT_L,
            border=1,
            box_size=2,
        )
        qr.add_data(url)
        qr.make(fit=True)
        qr_image = qr.make_image(fill_color="black", back_color="white").convert("1")
        label_text = "Escaneja"
        if token:
            label_text = f"Escaneja\n{token}"
        label = self._format_message(
            label_text,
            self.get_parameter("width").value,
            self.get_parameter("line_height").value,
            self._font,
        )
        self._show_qr_image(qr_image, label)

    def _extract_token(self, url: str) -> str:
        try:
            parsed = urlparse(url)
        except Exception:
            return ""
        query = parse_qs(parsed.query)
        token = query.get("token", [""])[0]
        return token.strip()

    def _show_qr_image(self, qr_image: "Image.Image", label_lines: List[str]) -> None:
        for display in (self._left, self._right):
            if display is not None:
                display.show_qr(qr_image, label_lines, self._font, self.get_parameter("line_height").value)

    def _on_anim(self, msg: String) -> None:
        raw = msg.data.strip()
        folder, loop_override = self._parse_anim_request(raw)
        if not folder:
            return
        # Avoid restarting the same animation on every repeated event.
        if (
            raw == self._current_anim
            and self._anim_thread is not None
            and self._anim_thread.is_alive()
        ):
            return
        self._stop_anim()
        self._anim_stop.clear()
        self._current_anim = raw
        self._anim_thread = threading.Thread(
            target=self._run_anim,
            args=(folder, loop_override),
            daemon=True,
        )
        self._anim_thread.start()

    def _parse_anim_request(self, raw: str) -> tuple[str, Optional[bool]]:
        if "|" not in raw:
            return raw, None
        folder, mode = raw.split("|", 1)
        folder = folder.strip()
        mode = mode.strip().lower()
        if mode in {"once", "one", "single"}:
            return folder, False
        if mode in {"loop", "repeat"}:
            return folder, True
        return folder, None

    def _stop_anim(self) -> None:
        if self._anim_thread and self._anim_thread.is_alive():
            self._anim_stop.set()
            self._anim_thread.join(timeout=1.0)
            if self._current_anim:
                self._publish_anim_state(f"stopped:{self._current_anim}")
        self._anim_thread = None
        self._current_anim = ""

    def _publish_anim_state(self, state: str) -> None:
        msg = String()
        msg.data = state
        self._anim_state_pub.publish(msg)

    def _run_anim(self, folder: str, loop_override: Optional[bool]) -> None:
        frames_left = self._load_frames(folder, "left")
        frames_right = self._load_frames(folder, "right")
        if not frames_left and not frames_right:
            self.get_logger().warning(f"No frames found for {folder}")
            self._publish_anim_state(f"done:{folder}")
            return

        delay = float(self.get_parameter("anim_delay").value)
        loop = bool(self.get_parameter("anim_loop").value) if loop_override is None else loop_override
        self._publish_anim_state(f"started:{folder}")
        while not self._anim_stop.is_set():
            max_len = max(len(frames_left), len(frames_right))
            if max_len == 0:
                break
            for idx in range(max_len):
                if self._anim_stop.is_set():
                    break
                if self._left is not None and frames_left:
                    self._left.show_image(frames_left[idx % len(frames_left)])
                if self._right is not None and frames_right:
                    self._right.show_image(frames_right[idx % len(frames_right)])
                self._anim_stop.wait(delay)
            if not loop:
                break
        if not self._anim_stop.is_set():
            self._publish_anim_state(f"done:{folder}")

    def _load_frames(self, folder: str, side: str) -> List["Image.Image"]:
        if Image is None:
            raise RuntimeError("Pillow is not installed")
        frame_dir = self._assets_root / folder / side
        if frame_dir in self._frame_cache:
            return self._frame_cache[frame_dir]
        if not frame_dir.exists():
            self._frame_cache[frame_dir] = []
            return []
        frames = []
        for path in sorted(frame_dir.iterdir()):
            if path.suffix.lower() not in (".png", ".bmp"):
                continue
            try:
                with Image.open(path) as img:
                    frames.append(img.convert("1").resize((
                        int(self.get_parameter("width").value),
                        int(self.get_parameter("height").value),
                    )))
            except Exception:
                continue
        self._frame_cache[frame_dir] = frames
        return frames


def main() -> None:
    rclpy.init()
    node = OledMessageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if isinstance(node, OledMessageNode):
            node._stop_anim()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
