import textwrap
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

        self._left = self._make_display(self.get_parameter("bus_left").value)
        self._right = self._make_display(self.get_parameter("bus_right").value)

        if self._left is None and self._right is None:
            self.get_logger().error("No OLED displays could be initialized.")
            return

        self._font = self._load_font()
        self._frame_cache: Dict[Path, List["Image.Image"]] = {}
        self._anim_thread: Optional[threading.Thread] = None
        self._anim_stop = threading.Event()

        self._show_text(self.get_parameter("message").value)

        self._text_sub = self.create_subscription(String, "oled_text", self._on_text, 10)
        self._anim_sub = self.create_subscription(String, "oled_anim", self._on_anim, 10)
        self.get_logger().info("OLED message node ready.")

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

    def _on_anim(self, msg: String) -> None:
        folder = msg.data.strip()
        if not folder:
            return
        self._stop_anim()
        self._anim_stop.clear()
        self._anim_thread = threading.Thread(
            target=self._run_anim,
            args=(folder,),
            daemon=True,
        )
        self._anim_thread.start()

    def _stop_anim(self) -> None:
        if self._anim_thread and self._anim_thread.is_alive():
            self._anim_stop.set()
            self._anim_thread.join(timeout=1.0)
        self._anim_thread = None

    def _run_anim(self, folder: str) -> None:
        frames_left = self._load_frames(folder, "left")
        frames_right = self._load_frames(folder, "right")
        if not frames_left and not frames_right:
            self.get_logger().warning(f"No frames found for {folder}")
            return

        delay = float(self.get_parameter("anim_delay").value)
        loop = bool(self.get_parameter("anim_loop").value)
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

    def _load_frames(self, folder: str, side: str) -> List["Image.Image"]:
        if Image is None:
            raise RuntimeError("Pillow is not installed")
        base = Path(self.get_parameter("assets_path").value)
        frame_dir = base / folder / side
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
