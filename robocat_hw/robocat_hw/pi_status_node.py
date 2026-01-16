import shutil
import socket
import subprocess
from typing import Optional, Tuple, Dict

import rclpy
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class PiStatusNode(Node):
    def __init__(self) -> None:
        super().__init__("pi_status_node")
        self.declare_parameter("publish_hz", 1.0)
        self.declare_parameter("disk_path", "/")
        self.declare_parameter("use_vcgencmd", True)

        self._publisher = self.create_publisher(DiagnosticArray, "pi_status", 10)
        self._prev_cpu_total = None
        self._prev_cpu_idle = None

        period = 1.0 / float(self.get_parameter("publish_hz").value)
        self._timer = self.create_timer(period, self._publish_status)

    def _publish_status(self) -> None:
        status = DiagnosticStatus()
        status.name = "raspberry_pi"
        status.hardware_id = socket.gethostname()
        status.level = DiagnosticStatus.OK
        status.message = "OK"

        for key, value in self._collect_metrics().items():
            status.values.append(KeyValue(key=key, value=value))

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(status)
        self._publisher.publish(array)

    def _collect_metrics(self) -> Dict[str, str]:
        metrics = {}

        cpu_temp_c = self._read_cpu_temp_c()
        if cpu_temp_c is not None:
            metrics["cpu_temp_c"] = f"{cpu_temp_c:.2f}"

        cpu_usage = self._read_cpu_usage_percent()
        if cpu_usage is not None:
            metrics["cpu_usage_percent"] = f"{cpu_usage:.2f}"

        load_avg = self._read_load_average()
        if load_avg is not None:
            metrics["load_1m"] = f"{load_avg[0]:.2f}"
            metrics["load_5m"] = f"{load_avg[1]:.2f}"
            metrics["load_15m"] = f"{load_avg[2]:.2f}"

        mem_used = self._read_mem_used_percent()
        if mem_used is not None:
            metrics["mem_used_percent"] = f"{mem_used:.2f}"

        disk_used = self._read_disk_used_percent()
        if disk_used is not None:
            metrics["disk_used_percent"] = f"{disk_used:.2f}"

        uptime_sec = self._read_uptime_sec()
        if uptime_sec is not None:
            metrics["uptime_sec"] = str(int(uptime_sec))

        cpu_freq_mhz = self._read_cpu_freq_mhz()
        if cpu_freq_mhz is not None:
            metrics["cpu_freq_mhz"] = f"{cpu_freq_mhz:.0f}"

        throttled = self._read_throttled()
        if throttled is not None:
            metrics["throttled_raw"] = throttled

        return metrics

    def _read_cpu_temp_c(self) -> Optional[float]:
        path = "/sys/class/thermal/thermal_zone0/temp"
        try:
            with open(path, "r", encoding="utf-8") as handle:
                milli_c = int(handle.read().strip())
            return milli_c / 1000.0
        except (OSError, ValueError):
            return None

    def _read_cpu_times(self) -> Optional[Tuple[int, int]]:
        try:
            with open("/proc/stat", "r", encoding="utf-8") as handle:
                line = handle.readline()
            parts = line.split()
            if not parts or parts[0] != "cpu":
                return None
            values = [int(value) for value in parts[1:]]
            total = sum(values)
            idle = values[3] + (values[4] if len(values) > 4 else 0)
            return total, idle
        except (OSError, ValueError, IndexError):
            return None

    def _read_cpu_usage_percent(self) -> Optional[float]:
        sample = self._read_cpu_times()
        if sample is None:
            return None
        total, idle = sample
        if self._prev_cpu_total is None or self._prev_cpu_idle is None:
            self._prev_cpu_total = total
            self._prev_cpu_idle = idle
            return None
        delta_total = total - self._prev_cpu_total
        delta_idle = idle - self._prev_cpu_idle
        self._prev_cpu_total = total
        self._prev_cpu_idle = idle
        if delta_total <= 0:
            return None
        return 100.0 * (delta_total - delta_idle) / delta_total

    def _read_load_average(self) -> Optional[Tuple[float, float, float]]:
        try:
            with open("/proc/loadavg", "r", encoding="utf-8") as handle:
                parts = handle.read().strip().split()
            return float(parts[0]), float(parts[1]), float(parts[2])
        except (OSError, ValueError, IndexError):
            return None

    def _read_mem_used_percent(self) -> Optional[float]:
        mem_total = None
        mem_available = None
        try:
            with open("/proc/meminfo", "r", encoding="utf-8") as handle:
                for line in handle:
                    if line.startswith("MemTotal:"):
                        mem_total = int(line.split()[1])
                    elif line.startswith("MemAvailable:"):
                        mem_available = int(line.split()[1])
                    if mem_total is not None and mem_available is not None:
                        break
            if mem_total is None or mem_available is None or mem_total == 0:
                return None
            used = mem_total - mem_available
            return 100.0 * used / mem_total
        except (OSError, ValueError):
            return None

    def _read_disk_used_percent(self) -> Optional[float]:
        disk_path = self.get_parameter("disk_path").value
        try:
            usage = shutil.disk_usage(disk_path)
            if usage.total == 0:
                return None
            return 100.0 * (usage.total - usage.free) / usage.total
        except OSError:
            return None

    def _read_uptime_sec(self) -> Optional[float]:
        try:
            with open("/proc/uptime", "r", encoding="utf-8") as handle:
                return float(handle.read().strip().split()[0])
        except (OSError, ValueError, IndexError):
            return None

    def _read_cpu_freq_mhz(self) -> Optional[float]:
        paths = [
            "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq",
            "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_cur_freq",
        ]
        for path in paths:
            try:
                with open(path, "r", encoding="utf-8") as handle:
                    khz = int(handle.read().strip())
                return khz / 1000.0
            except (OSError, ValueError):
                continue
        return None

    def _read_throttled(self) -> Optional[str]:
        if not self.get_parameter("use_vcgencmd").value:
            return None
        if shutil.which("vcgencmd") is None:
            return None
        try:
            result = subprocess.run(
                ["vcgencmd", "get_throttled"],
                check=False,
                capture_output=True,
                text=True,
            )
            output = result.stdout.strip()
            return output if output else None
        except OSError:
            return None


def main() -> None:
    rclpy.init()
    node = PiStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
