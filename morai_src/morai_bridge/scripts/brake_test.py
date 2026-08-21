#!/usr/bin/env python3

import argparse
import csv
import math
import socket
import struct
import time
from datetime import datetime
from pathlib import Path

import rclpy
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import VelocityReport
from rclpy.node import Node


HEADER = struct.Struct("<c12sci3i")
BODY = struct.Struct("<BBB5f")
TAIL = b"\r\n"
DETAIL_HEADER_SIZE = 27
DETAIL_BASE_SIZE = 152
DETAIL_VALUES = struct.Struct("<12f")


def ctrl_packet(accel, brake):
    body = BODY.pack(2, 4, 1, 0.0, 0.0, accel, brake, 0.0)
    return HEADER.pack(b"#", b"MoraiCtrlCmd", b"$", len(body), 1, 0, 0) + body + TAIL


def pulse_brake(elapsed, low, high, duty):
    return high if elapsed % 0.1 < 0.1 * duty else low


def parse_detail_packet(data):
    if len(data) < DETAIL_HEADER_SIZE + DETAIL_BASE_SIZE + DETAIL_VALUES.size + 2:
        raise ValueError(f"detail packet too short: {len(data)}")
    if data[:11] != b"#MoraiInfo$" or data[-2:] != TAIL:
        raise ValueError("invalid detail packet header or tail")
    data_length = struct.unpack_from("<I", data, 11)[0]
    if data_length < DETAIL_BASE_SIZE + DETAIL_VALUES.size:
        raise ValueError(f"detail payload too short: {data_length}")
    if len(data) != DETAIL_HEADER_SIZE + data_length + 2:
        raise ValueError(f"detail packet length mismatch: {len(data)} != {data_length + 29}")
    payload = data[DETAIL_HEADER_SIZE:DETAIL_HEADER_SIZE + data_length]
    sec, nanosec = struct.unpack_from("<II", payload)
    speed_kph = struct.unpack_from("<f", payload, 10)[0]
    values = DETAIL_VALUES.unpack_from(payload, DETAIL_BASE_SIZE)
    if not all(math.isfinite(value) for value in (speed_kph, *values)):
        raise ValueError("non-finite detail value")
    return sec, nanosec, speed_kph, values


def unique_csv_path(output_dir, brake, pulse, pulse_high, pulse_duty):
    output_dir.mkdir(parents=True, exist_ok=True)
    percent = round(pulse_duty * 100)
    mode = (
        f"pulse_{pulse_high:.2f}_{brake:.2f}_10hz_{percent}pct"
        if pulse else f"brake_{brake:.2f}")
    stem = mode.replace(".", "p") + datetime.now().strftime("_%Y%m%d_%H%M%S")
    path = output_dir / f"{stem}.csv"
    suffix = 1
    while path.exists():
        path = output_dir / f"{stem}_{suffix:02d}.csv"
        suffix += 1
    return path


class BrakeTest(Node):
    def __init__(self, args):
        super().__init__("brake_test")
        self.args = args
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.destination = (args.simulator_ip, args.port)
        self.detail_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.detail_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.detail_sock.bind((args.bind_ip, args.detail_port))
        self.detail_sock.setblocking(False)
        self.detail = None
        self.detail_received_at = None
        self.detail_announced = False
        self.speed = None
        self.acceleration = 0.0
        self.last_speed_sample = None
        self.control_conflict = False
        self.phase = "ARMING"
        self.started_at = time.monotonic()
        self.phase_started_at = self.started_at
        self.last_tick = self.started_at
        self.distance = 0.0
        self.brake_start_distance = 0.0
        self.peak_deceleration = 0.0
        self.done = False

        self.csv_path = unique_csv_path(
            Path(args.output_dir).expanduser(), args.brake, args.pulse,
            args.pulse_high, args.pulse_duty)
        self.csv_file = self.csv_path.open("x", newline="", buffering=1)
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            "elapsed_s", "phase", "speed_mps", "estimated_accel_mps2",
            "accel_cmd", "brake_cmd", "distance_m",
            "detail_stamp_sec", "detail_stamp_nanosec", "detail_speed_kph",
            "detail_age_s", *[f"detail_{index:02d}" for index in range(12)],
        ])

        self.create_subscription(
            VelocityReport, "/vehicle/status/velocity_status", self.on_velocity, 10)
        self.create_subscription(Control, "/control/command/control_cmd", self.on_other_control, 10)
        self.create_timer(1.0 / args.rate, self.tick)
        self.get_logger().info(
            f"arming: brake={args.brake:.2f}, target={args.target_speed:.1f} m/s, "
            f"detail_udp={args.bind_ip}:{args.detail_port}, csv={self.csv_path}")

    def read_detail(self, now):
        while True:
            try:
                data, _ = self.detail_sock.recvfrom(65535)
            except BlockingIOError:
                return
            try:
                self.detail = parse_detail_packet(data)
                self.detail_received_at = now
                if not self.detail_announced:
                    self.get_logger().info("receiving 1024 extended Ego status (12 raw floats)")
                    self.detail_announced = True
            except ValueError as error:
                self.get_logger().warning(str(error), throttle_duration_sec=5.0)

    def on_velocity(self, msg):
        now = time.monotonic()
        speed = float(msg.longitudinal_velocity)
        if not math.isfinite(speed):
            return
        if self.last_speed_sample is not None:
            old_speed, old_time = self.last_speed_sample
            dt = now - old_time
            if dt > 0.0:
                self.acceleration = (speed - old_speed) / dt
        self.last_speed_sample = (speed, now)
        self.speed = speed

    def on_other_control(self, _msg):
        self.control_conflict = True

    def send(self, accel, brake):
        self.sock.sendto(ctrl_packet(accel, brake), self.destination)

    def finish(self, reason):
        if self.done:
            return
        self.done = True
        self.phase = "DONE"
        self.get_logger().info(
            f"{reason}: brake={self.args.brake:.2f}, "
            f"braking_time={time.monotonic() - self.phase_started_at:.3f}s, "
            f"braking_distance={self.distance - self.brake_start_distance:.3f}m, "
            f"peak_deceleration={self.peak_deceleration:.3f}m/s^2, csv={self.csv_path}")

    def abort(self, reason):
        self.get_logger().error(reason)
        self.finish("aborted")

    def tick(self):
        now = time.monotonic()
        self.read_detail(now)
        dt = now - self.last_tick
        self.last_tick = now
        if self.speed is not None:
            self.distance += abs(self.speed) * dt

        if self.done:
            return
        if now - self.started_at > self.args.timeout:
            self.abort("test timeout")
            return
        if self.control_conflict:
            self.abort("/control/command/control_cmd is active; stop the competing controller first")
            return

        accel_cmd = 0.0
        brake_cmd = 0.0
        if self.phase == "ARMING":
            if now - self.started_at < 1.0:
                return
            if self.speed is None:
                self.abort("no /vehicle/status/velocity_status received")
                return
            if abs(self.speed) > 0.5:
                self.abort(f"vehicle must start stopped (current speed {self.speed:.2f} m/s)")
                return
            self.phase = "ACCEL"
            self.phase_started_at = now
            self.get_logger().info("full throttle")

        if self.phase == "ACCEL":
            accel_cmd = 1.0
            if self.speed >= self.args.target_speed:
                self.phase = "BRAKE"
                self.phase_started_at = now
                self.brake_start_distance = self.distance
                accel_cmd = 0.0
                brake_cmd = (
                    pulse_brake(
                        0.0, self.args.brake, self.args.pulse_high,
                        self.args.pulse_duty)
                    if self.args.pulse else self.args.brake)
                mode = (
                    f"pulse {self.args.pulse_high:.2f}/{self.args.brake:.2f} at 10 Hz "
                    f"({self.args.pulse_duty:.0%} high)"
                    if self.args.pulse else f"fixed brake {brake_cmd:.2f}")
                self.get_logger().info(f"target reached; {mode}")
        elif self.phase == "BRAKE":
            brake_cmd = (
                pulse_brake(
                    now - self.phase_started_at, self.args.brake,
                    self.args.pulse_high, self.args.pulse_duty)
                if self.args.pulse else self.args.brake)
            self.peak_deceleration = min(self.peak_deceleration, self.acceleration)
            if self.speed <= 0.2:
                self.finish("stopped")

        self.send(accel_cmd, brake_cmd)
        detail = (["", "", "", ""] + [""] * 12) if self.detail is None else [
            self.detail[0], self.detail[1], f"{self.detail[2]:.6f}",
            f"{now - self.detail_received_at:.6f}",
            *[f"{value:.9g}" for value in self.detail[3]],
        ]
        self.writer.writerow([
            f"{now - self.started_at:.6f}", self.phase,
            f"{self.speed:.6f}", f"{self.acceleration:.6f}",
            f"{accel_cmd:.2f}", f"{brake_cmd:.2f}", f"{self.distance:.6f}",
            *detail,
        ])

    def close(self):
        # Interrupt/timeout during acceleration must still leave a braking command behind.
        for _ in range(5):
            self.send(0.0, 1.0)
            time.sleep(0.02)
        self.csv_file.close()
        self.detail_sock.close()
        self.sock.close()


def parse_args():
    parser = argparse.ArgumentParser(description="MORAI brake stopping test")
    parser.add_argument("--brake", type=float, default=0.85, help="fixed brake or pulse low value, 0.0..1.0")
    parser.add_argument("--pulse", action="store_true", help="alternate --pulse-high and --brake at 10 Hz")
    parser.add_argument("--pulse-high", type=float, default=1.0, help="pulse high brake value")
    parser.add_argument("--pulse-duty", type=float, default=0.6, help="fraction of each pulse at --pulse-high")
    parser.add_argument("--target-speed", type=float, default=35.0, help="braking start speed in m/s")
    parser.add_argument("--simulator-ip", default="192.168.0.27")
    parser.add_argument("--port", type=int, default=9091)
    parser.add_argument("--bind-ip", default="192.168.0.37")
    parser.add_argument("--detail-port", type=int, default=1024)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--output-dir", default="brake_test_results")
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_known_args()


def self_test():
    assert len(ctrl_packet(1.0, 0.85)) == 55
    assert math.isclose(
        BODY.unpack(ctrl_packet(1.0, 0.85)[HEADER.size:-2])[6], 0.85, abs_tol=1e-6)
    assert pulse_brake(0.039, 0.62, 0.67, 0.4) == 0.67
    assert pulse_brake(0.041, 0.62, 0.67, 0.4) == 0.62
    assert pulse_brake(0.101, 0.62, 0.67, 0.4) == 0.67
    payload = bytearray(200)
    struct.pack_into("<IIbbf", payload, 0, 123, 456, 2, 4, 78.5)
    expected = tuple(float(value) for value in range(12))
    DETAIL_VALUES.pack_into(payload, DETAIL_BASE_SIZE, *expected)
    packet = b"#MoraiInfo$" + struct.pack("<I", len(payload)) + bytes(12) + payload + TAIL
    assert parse_detail_packet(packet) == (123, 456, 78.5, expected)
    print("brake_test self-test passed")


def main():
    args, ros_args = parse_args()
    if args.self_test:
        self_test()
        return
    if not 0.0 <= args.brake <= 1.0:
        raise SystemExit("--brake must be between 0.0 and 1.0")
    if not 0.0 <= args.pulse_high <= 1.0:
        raise SystemExit("--pulse-high must be between 0.0 and 1.0")
    if args.pulse and args.pulse_high < args.brake:
        raise SystemExit("--pulse-high must be greater than or equal to --brake")
    if not 0.0 <= args.pulse_duty <= 1.0:
        raise SystemExit("--pulse-duty must be between 0.0 and 1.0")
    if args.target_speed <= 0.0 or args.rate <= 0.0 or args.timeout <= 0.0:
        raise SystemExit("--target-speed, --rate and --timeout must be positive")

    rclpy.init(args=ros_args)
    node = BrakeTest(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.abort("interrupted")
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
