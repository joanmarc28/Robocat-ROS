import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

try:
    import smbus2
except ImportError:
    smbus2 = None

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None


class SensorsNode(Node):
    def __init__(self) -> None:
        super().__init__("sensors_node")
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("accel_enabled", True)
        self.declare_parameter("ultrasonic_enabled", True)
        self.declare_parameter("accel_bus", 6)
        self.declare_parameter("accel_address", 0x68)
        self.declare_parameter("ultrasonic_trig", 8)
        self.declare_parameter("ultrasonic_echo", 23)
        self.declare_parameter("ultrasonic_timeout_sec", 0.02)

        self._bus: Optional["smbus2.SMBus"] = None
        self._accel_addr: Optional[int] = None
        self._ultrasonic_ready = False

        self._publisher = self.create_publisher(DiagnosticArray, "robot_sensors", 10)

        self._init_accel()
        self._init_ultrasonic()

        period = 1.0 / float(self.get_parameter("publish_hz").value)
        self._timer = self.create_timer(period, self._publish)

    def _init_accel(self) -> None:
        if not bool(self.get_parameter("accel_enabled").value):
            return
        if smbus2 is None:
            self.get_logger().warning("smbus2 is not installed. Accelerometer disabled.")
            return
        bus_num = int(self.get_parameter("accel_bus").value)
        address = int(self.get_parameter("accel_address").value)
        try:
            self._bus = smbus2.SMBus(bus_num)
            self._accel_addr = address
            self._bus.write_byte_data(self._accel_addr, 0x6B, 0)
        except Exception as exc:
            self.get_logger().warning(f"Accelerometer init failed: {exc}")
            self._bus = None
            self._accel_addr = None

    def _init_ultrasonic(self) -> None:
        if not bool(self.get_parameter("ultrasonic_enabled").value):
            return
        if GPIO is None:
            self.get_logger().warning("RPi.GPIO is not installed. Ultrasonic disabled.")
            return
        trig = int(self.get_parameter("ultrasonic_trig").value)
        echo = int(self.get_parameter("ultrasonic_echo").value)
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(trig, GPIO.OUT)
            GPIO.setup(echo, GPIO.IN)
            self._ultrasonic_ready = True
        except Exception as exc:
            self.get_logger().warning(f"Ultrasonic init failed: {exc}")
            self._ultrasonic_ready = False

    def _read_raw(self, reg: int) -> Optional[int]:
        if self._bus is None or self._accel_addr is None:
            return None
        try:
            high = self._bus.read_byte_data(self._accel_addr, reg)
            low = self._bus.read_byte_data(self._accel_addr, reg + 1)
            value = (high << 8) | low
            if value > 32767:
                value -= 65536
            return value
        except Exception:
            return None

    def _read_accel(self) -> Optional[dict]:
        raw_x = self._read_raw(0x3B)
        raw_y = self._read_raw(0x3D)
        raw_z = self._read_raw(0x3F)
        if raw_x is None or raw_y is None or raw_z is None:
            return None
        return {
            "accel_x": raw_x / 16384.0,
            "accel_y": raw_y / 16384.0,
            "accel_z": raw_z / 16384.0,
        }

    def _read_gyro(self) -> Optional[dict]:
        raw_x = self._read_raw(0x43)
        raw_y = self._read_raw(0x45)
        raw_z = self._read_raw(0x47)
        if raw_x is None or raw_y is None or raw_z is None:
            return None
        return {
            "gyro_x": raw_x / 131.0,
            "gyro_y": raw_y / 131.0,
            "gyro_z": raw_z / 131.0,
        }

    def _read_temp(self) -> Optional[float]:
        raw_temp = self._read_raw(0x41)
        if raw_temp is None:
            return None
        return (raw_temp / 340.0) + 36.53

    def _read_pitch_roll(self, accel: dict) -> dict:
        pitch = math.degrees(
            math.atan2(accel["accel_y"], math.sqrt(accel["accel_x"] ** 2 + accel["accel_z"] ** 2))
        )
        roll = math.degrees(math.atan2(-accel["accel_x"], accel["accel_z"]))
        return {"pitch_deg": round(pitch, 2), "roll_deg": round(roll, 2)}

    def _read_ultrasonic(self) -> Optional[float]:
        if not self._ultrasonic_ready:
            return None
        trig = int(self.get_parameter("ultrasonic_trig").value)
        echo = int(self.get_parameter("ultrasonic_echo").value)
        timeout = float(self.get_parameter("ultrasonic_timeout_sec").value)
        try:
            GPIO.output(trig, False)
            time.sleep(0.0002)
            GPIO.output(trig, True)
            time.sleep(0.00001)
            GPIO.output(trig, False)

            start_time = time.time()
            start = start_time
            while GPIO.input(echo) == 0 and (time.time() - start_time) < timeout:
                start = time.time()
            if (time.time() - start_time) >= timeout:
                return None

            start_time = time.time()
            end = start_time
            while GPIO.input(echo) == 1 and (time.time() - start_time) < timeout:
                end = time.time()
            if (time.time() - start_time) >= timeout:
                return None

            duration = end - start
            distance_cm = (duration * 34300.0) / 2.0
            return round(distance_cm, 2)
        except Exception:
            return None

    def _publish(self) -> None:
        statuses = []

        accel_status = DiagnosticStatus()
        accel_status.name = "accelerometer"
        accel_status.hardware_id = "mpu6050"
        accel_status.level = DiagnosticStatus.OK
        accel_status.message = "OK"
        accel = self._read_accel()
        gyro = self._read_gyro()
        temp = self._read_temp()
        if accel is None or gyro is None:
            accel_status.level = DiagnosticStatus.WARN
            accel_status.message = "No data"
        else:
            for key, value in accel.items():
                accel_status.values.append(KeyValue(key=key, value=str(value)))
            for key, value in gyro.items():
                accel_status.values.append(KeyValue(key=key, value=str(value)))
            if temp is not None:
                accel_status.values.append(KeyValue(key="temp_c", value=str(round(temp, 2))))
            angles = self._read_pitch_roll(accel)
            for key, value in angles.items():
                accel_status.values.append(KeyValue(key=key, value=str(value)))
        statuses.append(accel_status)

        ultra_status = DiagnosticStatus()
        ultra_status.name = "ultrasonic"
        ultra_status.hardware_id = "hc_sr04"
        ultra_status.level = DiagnosticStatus.OK
        ultra_status.message = "OK"
        dist = self._read_ultrasonic()
        if dist is None:
            ultra_status.level = DiagnosticStatus.WARN
            ultra_status.message = "No echo"
            ultra_status.values.append(KeyValue(key="distance_cm", value="nan"))
        else:
            ultra_status.values.append(KeyValue(key="distance_cm", value=str(dist)))
        statuses.append(ultra_status)

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = statuses
        self._publisher.publish(array)


def main() -> None:
    rclpy.init()
    node = SensorsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if GPIO is not None:
            try:
                GPIO.cleanup()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
