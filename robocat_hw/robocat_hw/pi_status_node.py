import os
import platform
import re
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

        metrics["hostname"] = socket.gethostname()
        metrics["kernel"] = platform.release()

        os_name = self._read_os_release()
        if os_name is not None:
            metrics["os"] = os_name

        cpu_model = self._read_cpu_model()
        if cpu_model is not None:
            metrics["cpu_model"] = cpu_model

        cpu_cores = self._read_cpu_cores()
        if cpu_cores is not None:
            metrics["cpu_cores"] = str(cpu_cores)

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

        mem_totals = self._read_mem_totals_mb()
        if mem_totals is not None:
            metrics["mem_total_mb"] = str(mem_totals[0])
            metrics["mem_available_mb"] = str(mem_totals[1])

        swap_totals = self._read_swap_totals_mb()
        if swap_totals is not None:
            metrics["swap_total_mb"] = str(swap_totals[0])
            metrics["swap_free_mb"] = str(swap_totals[1])

        disk_used = self._read_disk_used_percent()
        if disk_used is not None:
            metrics["disk_used_percent"] = f"{disk_used:.2f}"

        disk_totals = self._read_disk_totals_gb()
        if disk_totals is not None:
            metrics["disk_total_gb"] = f"{disk_totals[0]:.2f}"
            metrics["disk_free_gb"] = f"{disk_totals[1]:.2f}"

        uptime_sec = self._read_uptime_sec()
        if uptime_sec is not None:
            metrics["uptime_sec"] = str(int(uptime_sec))

        cpu_freq_mhz = self._read_cpu_freq_mhz()
        if cpu_freq_mhz is not None:
            metrics["cpu_freq_mhz"] = f"{cpu_freq_mhz:.0f}"

        throttled = self._read_throttled()
        if throttled is not None:
            metrics["throttled_raw"] = throttled

        vcgencmd_mem = self._read_vcgencmd_mem()
        if vcgencmd_mem is not None:
            metrics.update(vcgencmd_mem)

        net_summary = self._read_net_summary()
        if net_summary is not None:
            metrics.update(net_summary)

        wifi_signal = self._read_wifi_signal_dbm()
        if wifi_signal is not None:
            metrics["wifi_signal_dbm"] = str(wifi_signal)

        temps = self._read_hwmon_temps()
        if temps is not None:
            metrics.update(temps)

        voltages = self._read_hwmon_voltages()
        if voltages is not None:
            metrics.update(voltages)

        freqs = self._read_hwmon_freqs()
        if freqs is not None:
            metrics.update(freqs)

        return metrics

    def _read_os_release(self) -> Optional[str]:
        try:
            with open("/etc/os-release", "r", encoding="utf-8") as handle:
                for line in handle:
                    if line.startswith("PRETTY_NAME="):
                        return line.split("=", 1)[1].strip().strip('"')
            return None
        except OSError:
            return None

    def _read_cpu_model(self) -> Optional[str]:
        try:
            with open("/proc/cpuinfo", "r", encoding="utf-8") as handle:
                for line in handle:
                    if line.lower().startswith("model name"):
                        return line.split(":", 1)[1].strip()
            return None
        except OSError:
            return None

    def _read_cpu_cores(self) -> Optional[int]:
        try:
            return os.cpu_count()
        except OSError:
            return None

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

    def _read_mem_totals_mb(self) -> Optional[Tuple[int, int]]:
        mem_total = None
        mem_available = None
        try:
            with open("/proc/meminfo", "r", encoding="utf-8") as handle:
                for line in handle:
                    if line.startswith("MemTotal:"):
                        mem_total = int(line.split()[1]) // 1024
                    elif line.startswith("MemAvailable:"):
                        mem_available = int(line.split()[1]) // 1024
                    if mem_total is not None and mem_available is not None:
                        break
            if mem_total is None or mem_available is None:
                return None
            return mem_total, mem_available
        except (OSError, ValueError):
            return None

    def _read_swap_totals_mb(self) -> Optional[Tuple[int, int]]:
        swap_total = None
        swap_free = None
        try:
            with open("/proc/meminfo", "r", encoding="utf-8") as handle:
                for line in handle:
                    if line.startswith("SwapTotal:"):
                        swap_total = int(line.split()[1]) // 1024
                    elif line.startswith("SwapFree:"):
                        swap_free = int(line.split()[1]) // 1024
                    if swap_total is not None and swap_free is not None:
                        break
            if swap_total is None or swap_free is None:
                return None
            return swap_total, swap_free
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

    def _read_disk_totals_gb(self) -> Optional[Tuple[float, float]]:
        disk_path = self.get_parameter("disk_path").value
        try:
            usage = shutil.disk_usage(disk_path)
            if usage.total == 0:
                return None
            total_gb = usage.total / (1024 ** 3)
            free_gb = usage.free / (1024 ** 3)
            return total_gb, free_gb
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

    def _read_vcgencmd_mem(self) -> Optional[Dict[str, str]]:
        if not self.get_parameter("use_vcgencmd").value:
            return None
        if shutil.which("vcgencmd") is None:
            return None
        metrics = {}
        for target in ("gpu", "arm"):
            try:
                result = subprocess.run(
                    ["vcgencmd", "get_mem", target],
                    check=False,
                    capture_output=True,
                    text=True,
                )
                output = result.stdout.strip()
                if not output:
                    continue
                parts = output.split("=", 1)
                if len(parts) != 2:
                    continue
                value = parts[1].strip()
                metrics[f"mem_{target}"] = value
            except OSError:
                continue
        return metrics or None

    def _read_net_summary(self) -> Optional[Dict[str, str]]:
        ifaces = self._read_net_ifaces()
        if not ifaces:
            return None
        metrics = {}
        metrics["net_ifaces"] = ",".join(ifaces)

        ip_summary = self._read_ip_addrs()
        if ip_summary:
            metrics["ip_addrs"] = ip_summary

        rx_tx = self._read_net_bytes()
        if rx_tx:
            metrics["net_rx_bytes"] = str(rx_tx[0])
            metrics["net_tx_bytes"] = str(rx_tx[1])

        return metrics

    def _read_net_ifaces(self) -> Optional[list]:
        try:
            entries = os.listdir("/sys/class/net")
            ifaces = [name for name in entries if name != "lo"]
            return ifaces
        except OSError:
            return None

    def _read_ip_addrs(self) -> Optional[str]:
        if shutil.which("ip") is None:
            return None
        try:
            result = subprocess.run(
                ["ip", "-o", "-4", "addr", "show"],
                check=False,
                capture_output=True,
                text=True,
            )
            addrs = []
            for line in result.stdout.splitlines():
                parts = line.split()
                if len(parts) >= 4:
                    iface = parts[1]
                    ip = parts[3]
                    if iface != "lo":
                        addrs.append(f"{iface}:{ip}")
            return ",".join(addrs) if addrs else None
        except OSError:
            return None

    def _read_net_bytes(self) -> Optional[Tuple[int, int]]:
        rx_total = 0
        tx_total = 0
        try:
            with open("/proc/net/dev", "r", encoding="utf-8") as handle:
                lines = handle.read().strip().splitlines()[2:]
            for line in lines:
                name, stats = line.split(":", 1)
                iface = name.strip()
                if iface == "lo":
                    continue
                parts = stats.split()
                if len(parts) < 9:
                    continue
                rx_total += int(parts[0])
                tx_total += int(parts[8])
            return rx_total, tx_total
        except (OSError, ValueError, IndexError):
            return None

    def _read_wifi_signal_dbm(self) -> Optional[int]:
        if shutil.which("iw") is None:
            return None
        ifaces = self._read_net_ifaces()
        if not ifaces:
            return None
        for iface in ifaces:
            try:
                result = subprocess.run(
                    ["iw", "dev", iface, "link"],
                    check=False,
                    capture_output=True,
                    text=True,
                )
                match = re.search(r"signal:\s*(-?\d+)\s*dBm", result.stdout)
                if match:
                    return int(match.group(1))
            except OSError:
                continue
        return None

    def _read_hwmon_temps(self) -> Optional[Dict[str, str]]:
        base = "/sys/class/hwmon"
        try:
            entries = os.listdir(base)
        except OSError:
            return None
        metrics = {}
        for entry in entries:
            name_path = os.path.join(base, entry, "name")
            try:
                with open(name_path, "r", encoding="utf-8") as handle:
                    chip = handle.read().strip()
            except OSError:
                chip = entry
            for idx in range(1, 10):
                temp_path = os.path.join(base, entry, f"temp{idx}_input")
                try:
                    with open(temp_path, "r", encoding="utf-8") as handle:
                        milli_c = int(handle.read().strip())
                    metrics[f"temp_{chip}_{idx}_c"] = f"{milli_c / 1000.0:.2f}"
                except (OSError, ValueError):
                    continue
        return metrics or None

    def _read_hwmon_voltages(self) -> Optional[Dict[str, str]]:
        base = "/sys/class/hwmon"
        try:
            entries = os.listdir(base)
        except OSError:
            return None
        metrics = {}
        for entry in entries:
            name_path = os.path.join(base, entry, "name")
            try:
                with open(name_path, "r", encoding="utf-8") as handle:
                    chip = handle.read().strip()
            except OSError:
                chip = entry
            for idx in range(0, 16):
                volt_path = os.path.join(base, entry, f"in{idx}_input")
                try:
                    with open(volt_path, "r", encoding="utf-8") as handle:
                        micro_v = int(handle.read().strip())
                    metrics[f"volt_{chip}_{idx}_v"] = f"{micro_v / 1_000_000.0:.3f}"
                except (OSError, ValueError):
                    continue
        return metrics or None

    def _read_hwmon_freqs(self) -> Optional[Dict[str, str]]:
        base = "/sys/class/hwmon"
        try:
            entries = os.listdir(base)
        except OSError:
            return None
        metrics = {}
        for entry in entries:
            name_path = os.path.join(base, entry, "name")
            try:
                with open(name_path, "r", encoding="utf-8") as handle:
                    chip = handle.read().strip()
            except OSError:
                chip = entry
            for idx in range(1, 10):
                freq_path = os.path.join(base, entry, f"freq{idx}_input")
                try:
                    with open(freq_path, "r", encoding="utf-8") as handle:
                        hz = int(handle.read().strip())
                    metrics[f"freq_{chip}_{idx}_hz"] = str(hz)
                except (OSError, ValueError):
                    continue
        return metrics or None


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
