#!/usr/bin/python3

import copy
import math
import socket
import struct
import time

import rclpy
from autoware_vehicle_msgs.msg import VelocityReport
from morai_msgs.msg import CollisionData, ObjectStatus
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan, NavSatFix, NavSatStatus


COLLISION = struct.Struct("<15si3iii" + "hh6f" * 5 + "2s")
IMU = struct.Struct("<9si3iii10d2s")
LIDAR_HEADER = struct.Struct("<9si3f")
LIDAR_POINT = struct.Struct("<Hb")

assert COLLISION.size == 181
assert IMU.size == 115
assert LIDAR_HEADER.size + LIDAR_POINT.size * 360 + 2 == 1107


def nmea_coordinate(value, direction):
    raw = float(value)
    result = int(raw / 100) + (raw % 100) / 60
    return -result if direction in ("S", "W") else result


def apply_yaw_offset(w, x, y, z, yaw_offset_rad):
    half = yaw_offset_rad * 0.5
    c = math.cos(half)
    s = math.sin(half)
    return (
        c * w - s * z,
        c * x - s * y,
        c * y + s * x,
        c * z + s * w,
    )


class MoraiSensorReceiver(Node):
    def __init__(self):
        super().__init__("morai_sensor_receiver_node")

        self.declare_parameter("bind_ip", "192.168.0.10")
        self.declare_parameter("collision_port", 9011)
        self.declare_parameter("lidar_port", 19005)
        self.declare_parameter("gnss_port", 9006)
        self.declare_parameter("imu_port", 9007)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("lidar_frame", "lidar")
        self.declare_parameter("gnss_frame", "base_link")
        self.declare_parameter("imu_frame", "imu")
        self.declare_parameter("slam_mode", True)
        self.declare_parameter("imu_yaw_offset_deg", -1.347045)
        self.declare_parameter("imu_timeout_sec", 0.2)
        self.declare_parameter("velocity_timeout_sec", 0.2)

        self.imu_timeout_sec = self.get_parameter("imu_timeout_sec").value
        self.velocity_timeout_sec = self.get_parameter("velocity_timeout_sec").value
        for name in ("imu_timeout_sec", "velocity_timeout_sec"):
            value = self.get_parameter(name).value
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")

        bind_ip = self.get_parameter("bind_ip").value
        self.map_frame = self.get_parameter("map_frame").value
        self.lidar_frame = self.get_parameter("lidar_frame").value
        self.gnss_frame = self.get_parameter("gnss_frame").value
        self.imu_frame = self.get_parameter("imu_frame").value
        self.slam_mode = self.get_parameter("slam_mode").value
        self.imu_yaw_offset_deg = self.get_parameter("imu_yaw_offset_deg").value
        self.imu_yaw_offset_rad = math.radians(self.imu_yaw_offset_deg)
        ports = {
            "collision": self.get_parameter("collision_port").value,
            "lidar": self.get_parameter("lidar_port").value,
            "gnss": self.get_parameter("gnss_port").value,
            "imu": self.get_parameter("imu_port").value,
        }

        self.sockets = {
            name: self._open_socket(bind_ip, port) for name, port in ports.items()
        }
        self.handlers = {
            "collision": self._publish_collision,
            "lidar": self._publish_lidar,
            "gnss": self._publish_gnss,
            "imu": self._publish_imu,
        }

        self.collision_pub = self.create_publisher(CollisionData, "/CollisionData", 10)
        self.lidar_pub = self.create_publisher(
            LaserScan, "/scan", qos_profile_sensor_data
        )
        self.gnss_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        self.imu_time_offset_ns = None
        self.last_imu_packet_stamp_ns = None
        self.imu_rate_started = time.monotonic()
        self.imu_raw_count = 0
        self.imu_publish_count = 0
        self.imu_fallback_count = 0
        self.last_imu_receive_time = None
        self.last_velocity_receive_time = None
        self.last_real_imu = None
        self.imu_fallback_active = False
        # Public velocity includes synthetic zeros; only real UDP may refresh this clock.
        self.velocity_sub = self.create_subscription(
            VelocityReport, "/morai_bridge/vehicle_status/velocity_raw",
            self._receive_raw_velocity, 1
        )
        self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.imu_watchdog_timer = self.create_timer(
            0.02, self._check_imu_timeout, clock=self.watchdog_clock
        )
        self.last_altitude = 0.0
        self.timer = self.create_timer(0.001, self._poll)
        self.imu_rate_timer = self.create_timer(10.0, self._report_imu_rate)

        self.get_logger().info(
            "Listening on %s: collision=%d lidar=%d gnss=%d imu=%d"
            % (
                bind_ip,
                ports["collision"],
                ports["lidar"],
                ports["gnss"],
                ports["imu"],
            )
        )
        self.get_logger().info(
            f"IMU orientation yaw offset: {self.imu_yaw_offset_deg:+.6f} deg"
        )

    def destroy_node(self):
        for sock in self.sockets.values():
            sock.close()
        super().destroy_node()

    @staticmethod
    def _open_socket(ip, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2**20)
        sock.bind((ip, port))
        sock.setblocking(False)
        return sock

    def _poll(self):
        for name, sock in self.sockets.items():
            for _ in range(32):
                try:
                    data, _ = sock.recvfrom(65535)
                except BlockingIOError:
                    break
                try:
                    self.handlers[name](data)
                except (UnicodeError, ValueError, IndexError, struct.error) as error:
                    self.get_logger().warning(
                        f"Invalid {name} packet: {error}", throttle_duration_sec=5.0
                    )

    def _publish_collision(self, data):
        if len(data) != COLLISION.size:
            raise ValueError(f"expected {COLLISION.size} bytes, got {len(data)}")
        values = COLLISION.unpack(data)
        if not values[0].startswith(b"#") or values[-1] != b"\r\n":
            raise ValueError("bad header or tail")

        msg = CollisionData()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        objects = [values[7 + i * 8 : 15 + i * 8] for i in range(5)]
        count = min(max(values[2], 0), 5)
        if count == 0:
            count = sum(any(value != 0 for value in item) for item in objects)

        if count:
            msg.global_offset_x, msg.global_offset_y, msg.global_offset_z = objects[0][5:8]
        for obj_type, obj_id, x, y, z, _, _, _ in objects[:count]:
            obj = ObjectStatus()
            obj.type = obj_type
            obj.unique_id = obj_id
            obj.position.x = x
            obj.position.y = y
            obj.position.z = z
            msg.collision_object.append(obj)
        self.collision_pub.publish(msg)

    def _publish_lidar(self, data):
        if len(data) != 1107:
            raise ValueError(f"expected 1107 bytes, got {len(data)}")

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.lidar_frame
        msg.angle_min = math.pi / 2
        msg.angle_increment = 2 * math.pi / 360
        msg.angle_max = msg.angle_min + msg.angle_increment * 359
        msg.range_min = 0.0
        msg.range_max = 10.0

        offset = LIDAR_HEADER.size
        for index in range(360):
            distance_mm, intensity = LIDAR_POINT.unpack_from(
                data, offset + index * LIDAR_POINT.size
            )
            distance = distance_mm / 1000.0
            msg.ranges.append(distance if 0.0 < distance < msg.range_max else math.inf)
            msg.intensities.append(float(intensity))
        self.lidar_pub.publish(msg)

    def _publish_gnss(self, data):
        text = data.rstrip(b"\x00").decode("ascii", errors="strict").strip()
        for sentence in text.splitlines():
            fields = sentence.split(",")
            if fields[0] == "$GPGGA" and len(fields) > 10:
                latitude = nmea_coordinate(fields[2], fields[3])
                longitude = nmea_coordinate(fields[4], fields[5])
                self.last_altitude = float(fields[9])
                has_fix = fields[6] not in ("", "0")
            else:
                continue

            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.gnss_frame
            msg.status.status = (
                NavSatStatus.STATUS_FIX if has_fix else NavSatStatus.STATUS_NO_FIX
            )
            msg.status.service = NavSatStatus.SERVICE_GPS
            msg.latitude = latitude
            msg.longitude = longitude
            msg.altitude = self.last_altitude
            self.gnss_pub.publish(msg)

    def _publish_imu(self, data):
        if len(data) != IMU.size:
            raise ValueError(f"expected {IMU.size} bytes, got {len(data)}")
        values = IMU.unpack(data)
        if not values[0].startswith(b"#") or values[-1] != b"\r\n":
            raise ValueError("bad header or tail")
        if not all(math.isfinite(value) for value in values[7:17]):
            raise ValueError("nonfinite IMU orientation, angular velocity or acceleration")

        self.imu_raw_count += 1
        msg = Imu()
        packet_stamp_ns = values[5] * 1_000_000_000 + values[6]
        if packet_stamp_ns == self.last_imu_packet_stamp_ns:
            return
        if self.last_imu_packet_stamp_ns is not None:
            gap = (packet_stamp_ns - self.last_imu_packet_stamp_ns) / 1e9
            if gap > 0.05:
                self.get_logger().warning(f"Raw IMU UDP gap: {gap:.3f} s")
        self.last_imu_packet_stamp_ns = packet_stamp_ns
        now_ns = self.get_clock().now().nanoseconds
        if self.slam_mode:
            if (
                self.imu_time_offset_ns is None
                or abs(packet_stamp_ns + self.imu_time_offset_ns - now_ns)
                > 1_000_000_000
            ):
                self.imu_time_offset_ns = now_ns - packet_stamp_ns
                self.get_logger().info(
                    f"IMU timestamp offset: {self.imu_time_offset_ns / 1e9:.6f} s"
                )
            stamp_ns = packet_stamp_ns + self.imu_time_offset_ns
        else:
            stamp_ns = now_ns
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(
            stamp_ns, 1_000_000_000
        )
        msg.header.frame_id = self.imu_frame
        msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = (
            apply_yaw_offset(*values[7:11], self.imu_yaw_offset_rad)
        )
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = values[11:14]
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = values[14:17]
        self.last_imu_receive_time = time.monotonic()
        self.last_real_imu = msg
        self.imu_pub.publish(msg)
        self.imu_publish_count += 1

    def _receive_raw_velocity(self, msg):
        if all(math.isfinite(value) for value in (
            msg.longitudinal_velocity, msg.lateral_velocity, msg.heading_rate
        )):
            self.last_velocity_receive_time = time.monotonic()

    def _check_imu_timeout(self):
        # Never invent an initial attitude or infer a simulator pause from IMU loss alone.
        if self.last_real_imu is None or self.last_velocity_receive_time is None:
            return
        now = time.monotonic()
        imu_age = now - self.last_imu_receive_time
        velocity_age = now - self.last_velocity_receive_time
        if imu_age <= self.imu_timeout_sec or velocity_age <= self.velocity_timeout_sec:
            if self.imu_fallback_active:
                self.get_logger().info("Real input resumed; stopped synthetic zero-angular IMU")
                self.imu_fallback_active = False
            return

        # shortcut: simultaneous UDP loss is treated as simulator pause, not proof of a stop.
        # Keep the last attitude/acceleration (including gravity); only zero angular velocity.
        # This is synthetic data, and must never refresh either real-input receive clock.
        msg = copy.deepcopy(self.last_real_imu)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0
        self.imu_pub.publish(msg)
        self.imu_fallback_active = True
        self.imu_fallback_count += 1
        self.get_logger().warning(
            f"IMU and vehicle UDP stale ({imu_age:.3f}s / {velocity_age:.3f}s); "
            "publishing synthetic zero-angular IMU for simulator pause",
            throttle_duration_sec=2.0, throttle_time_source_type=self.watchdog_clock
        )

    def _report_imu_rate(self):
        elapsed = time.monotonic() - self.imu_rate_started
        self.get_logger().info(
            f"IMU rates: raw_udp={self.imu_raw_count / elapsed:.2f} Hz, "
            f"published={self.imu_publish_count / elapsed:.2f} Hz, "
            f"synthetic={self.imu_fallback_count / elapsed:.2f} Hz"
        )
        self.imu_rate_started = time.monotonic()
        self.imu_raw_count = 0
        self.imu_publish_count = 0
        self.imu_fallback_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = MoraiSensorReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
