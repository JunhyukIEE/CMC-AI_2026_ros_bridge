#!/usr/bin/env python3
"""Exercise the real bridge executables using isolated ROS topics and loopback UDP only."""

import argparse
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
import unittest

from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import VelocityReport
from sensor_msgs.msg import Imu


def vehicle_packet(speed_mps):
    """Match EgoVehicleStatusUDP, including the 24 floats after map_data_id."""
    stamp = time.time_ns()
    values = [0.0] * 24
    values[14] = speed_mps * 3.6  # velocity_x, km/h
    body = struct.pack(
        '<iibbfi24f38s', stamp // 10**9, stamp % 10**9, 2, 4,
        speed_mps * 3.6, 0, *values, bytes(38))
    return b'#MoraiInfo$' + struct.pack('<i', len(body)) + bytes(12) + body + b'\r\n'


def imu_packet(angular=(0.1, -0.2, 0.3)):
    stamp = time.time_ns()
    return struct.pack(
        '<9si3iii10d2s', b'#IMUData$', 88, 0, 0, 0,
        stamp // 10**9, stamp % 10**9,
        1.0, 0.0, 0.0, 0.0, *angular, 0.4, -0.5, 9.7, b'\r\n')


class UdpWatchdogTest(unittest.TestCase):
    """No simulator connection, real vehicle control, or existing node restart."""

    @classmethod
    def setUpClass(cls):
        cls.temp_dir = tempfile.TemporaryDirectory(prefix='morai-watchdog-')
        cls.processes = []
        cls.logs = []
        cls.node = None
        cls.sockets = []
        cls.addClassCleanup(cls.cleanup)
        # Unique remaps also isolate tests from anything else in the selected domain.
        os.environ['ROS_DOMAIN_ID'] = str(170 + os.getpid() % 30)
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        rclpy.init(args=[])
        cls.node = rclpy.create_node(f'watchdog_test_{os.getpid()}')
        prefix = f'/watchdog_test_{os.getpid()}'
        topics = [
            '/vehicle/status/velocity_status', '/morai_bridge/vehicle_status/velocity_raw',
            '/vehicle/status/steering_status', '/vehicle/status/gear_status',
            '/vehicle/status/control_mode', '/system/operation_mode/availability',
            '/control/command/control_cmd', '/control/command/gear_cmd',
            '/imu/data', '/CollisionData', '/scan', '/gps/fix',
        ]
        remaps = []
        for topic in topics:
            remaps += ['-r', f'{topic}:={prefix}{topic}']
        cls.public_samples = []
        cls.raw_samples = []
        cls.commands = []
        cls.imu_samples = []
        cls.public_sub = cls.node.create_subscription(
            VelocityReport, prefix + topics[0],
            lambda msg: cls.public_samples.append((time.monotonic(), msg)), 100)
        cls.raw_sub = cls.node.create_subscription(
            VelocityReport, prefix + topics[1],
            lambda msg: cls.raw_samples.append((time.monotonic(), msg)), 100)
        cls.control_pub = cls.node.create_publisher(Control, prefix + topics[6], 10)
        cls.imu_sub = cls.node.create_subscription(
            Imu, prefix + '/imu/data',
            lambda msg: cls.imu_samples.append((time.monotonic(), msg)), 100)
        cls.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        cls.sockets.append(cls.command_socket)
        cls.command_socket.bind(('127.0.0.1', 0))
        cls.command_socket.setblocking(False)
        cls.status_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        cls.sockets.append(cls.status_socket)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserved:
            reserved.bind(('127.0.0.1', 0))
            cls.status_port = reserved.getsockname()[1]
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserved:
            reserved.bind(('127.0.0.1', 0))
            cls.imu_port = reserved.getsockname()[1]

        receiver_args = ['udp_port:=' + str(cls.status_port), 'velocity_timeout_sec:=0.2']
        sender_args = [
            'simulator_ip:=127.0.0.1',
            'cmd_udp_port:=' + str(cls.command_socket.getsockname()[1]),
            'velocity_timeout_sec:=0.2', 'speed_debug:=false',
            'speed_kp:=0.75', 'speed_ki:=0.01', 'speed_kd:=0.03',
            'max_throttle_command:=1.0', 'max_brake_command:=0.63',
        ]
        sensor_args = [
            'bind_ip:=127.0.0.1', 'imu_port:=' + str(cls.imu_port),
            'collision_port:=0', 'lidar_port:=0', 'gnss_port:=0',
            'slam_mode:=false', 'imu_yaw_offset_deg:=0.0',
            'imu_timeout_sec:=0.2', 'velocity_timeout_sec:=0.2',
        ]
        for name, executable, parameters in (
            ('receiver', EXECUTABLES.receiver, receiver_args),
            ('sender', EXECUTABLES.sender, sender_args),
            ('sensor', EXECUTABLES.sensor, sensor_args),
        ):
            log = open(Path(cls.temp_dir.name) / f'{name}.log', 'w+')
            cls.logs.append(log)
            args = ([sys.executable, executable] if name == 'sensor' else [executable])
            args += ['--ros-args', *remaps]
            for parameter in parameters:
                args += ['-p', parameter]
            cls.processes.append(subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT))

        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.02)
            if any(process.poll() is not None for process in cls.processes):
                raise RuntimeError('Bridge test process exited before discovery')
            if (cls.control_pub.get_subscription_count() and
                    cls.node.count_publishers(prefix + topics[1]) and
                    cls.node.count_publishers(prefix + topics[0]) and
                    cls.node.count_publishers(prefix + '/imu/data') and
                    cls.node.count_subscribers(prefix + topics[1]) >= 3):
                break
        else:
            raise RuntimeError('Timed out discovering isolated bridge nodes')

    @classmethod
    def cleanup(cls):
        for process in cls.processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        for process in cls.processes:
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
        for log in cls.logs:
            log.seek(0)
            print(log.read())
            log.close()
        if cls.node is not None:
            cls.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sock in cls.sockets:
            sock.close()
        cls.temp_dir.cleanup()

    def run_for(self, seconds, speed=None, target=None, malformed=False,
                imu=False, imu_payload=None):
        start = time.monotonic()
        next_input = start
        while time.monotonic() - start < seconds:
            now = time.monotonic()
            if now >= next_input:
                if speed is not None or malformed:
                    data = b'bad' if malformed else vehicle_packet(speed)
                    self.status_socket.sendto(data, ('127.0.0.1', self.status_port))
                if target is not None:
                    msg = Control()
                    msg.longitudinal.velocity = target
                    msg.lateral.steering_tire_angle = 0.1
                    self.control_pub.publish(msg)
                if imu or imu_payload is not None:
                    data = imu_packet() if imu_payload is None else imu_payload
                    self.last_sent_imu = data
                    self.status_socket.sendto(data, ('127.0.0.1', self.imu_port))
                next_input = now + 0.02
            rclpy.spin_once(self.node, timeout_sec=0.002)
            while True:
                try:
                    data, _ = self.command_socket.recvfrom(2048)
                except BlockingIOError:
                    break
                self.assertEqual(data[:14], b'#MoraiCtrlCmd$')
                self.assertEqual(data[-2:], b'\r\n')
                command = struct.unpack('<BBB5f', data[30:-2])
                self.assertEqual(command[:3], (2, 4, 1))
                self.assertEqual(command[3:5], (0.0, 0.0))
                self.assertGreaterEqual(command[5], 0.0)
                self.assertLessEqual(command[5], 1.0)
                self.assertGreaterEqual(command[6], 0.0)
                self.assertLessEqual(command[6], 0.630001)
                self.assertFalse(command[5] > 0 and command[6] > 0)
                self.commands.append((time.monotonic(), command))
            for process in self.processes:
                self.assertIsNone(process.poll(), 'Bridge test process exited')
        return start

    def assert_zero_fallback(self, start):
        # Allow the 0.2 s timeout, 20 ms timer and local DDS scheduling margin.
        cutoff = start + 0.35
        public = [msg for t, msg in self.public_samples if t > cutoff]
        commands = [cmd for t, cmd in self.commands if t > cutoff]
        self.assertGreater(len(public), 3)
        self.assertGreater(len(commands), 3)
        self.assertFalse([msg for t, msg in self.raw_samples if t > cutoff])
        for msg in public:
            self.assertEqual((msg.longitudinal_velocity, msg.lateral_velocity, msg.heading_rate),
                             (0.0, 0.0, 0.0))
            self.assertEqual(msg.header.frame_id, 'base_link')
        stamps = [(msg.header.stamp.sec, msg.header.stamp.nanosec) for msg in public]
        self.assertEqual(len(stamps), len(set(stamps)))
        for cmd in commands:
            self.assertEqual(cmd[5:7], (0.0, 0.0))
            self.assertAlmostEqual(cmd[7], 0.1 / math.radians(40), places=5)

    def assert_imu_fallback(self, start):
        samples = [msg for t, msg in self.imu_samples if t > start + 0.35]
        self.assertGreater(len(samples), 3)
        for msg in samples:
            self.assertEqual((msg.angular_velocity.x, msg.angular_velocity.y,
                              msg.angular_velocity.z), (0.0, 0.0, 0.0))
            # Do not reset orientation or erase the existing acceleration/gravity vector.
            self.assertEqual((msg.orientation.w, msg.orientation.x, msg.orientation.y,
                              msg.orientation.z), (1.0, 0.0, 0.0, 0.0))
            self.assertEqual((msg.linear_acceleration.x, msg.linear_acceleration.y,
                              msg.linear_acceleration.z), (0.4, -0.5, 9.7))
            self.assertEqual(msg.header.frame_id, 'imu')
            self.assertEqual(list(msg.angular_velocity_covariance), [0.0] * 9)
        stamps = [msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec for msg in samples]
        self.assertTrue(all(a < b for a, b in zip(stamps, stamps[1:])))
        self.assertLess(abs(time.time_ns() - stamps[-1]), 200_000_000)

    def assert_real_imu(self, start):
        samples = [msg for t, msg in self.imu_samples if t > start + 0.1]
        self.assertGreater(len(samples), 3)
        for msg in samples:
            self.assertEqual((msg.angular_velocity.x, msg.angular_velocity.y,
                              msg.angular_velocity.z), (0.1, -0.2, 0.3))

    def test_loss_and_recovery(self):
        with self.subTest('startup does not take control before control_cmd'):
            self.run_for(0.3)
            self.assertFalse(self.commands)
            self.assertFalse(self.imu_samples)

        with self.subTest('no synthetic IMU before any real vehicle input'):
            start = self.run_for(0.3, imu=True)
            self.assert_real_imu(start)
            start = self.run_for(0.5)
            self.assertFalse([msg for t, msg in self.imu_samples if t > start + 0.3])

        with self.subTest('normal UDP publishes matching real and public velocity'):
            start = self.run_for(0.6, speed=10.0, target=10.0)
            raw = [msg for t, msg in self.raw_samples if t > start + 0.3]
            public = [msg for t, msg in self.public_samples if t > start + 0.3]
            self.assertGreater(len(raw), 3)
            self.assertGreater(len(public), 3)
            for msg in raw + public:
                self.assertAlmostEqual(msg.longitudinal_velocity, 10.0, places=4)
            raw_stamps = {(msg.header.stamp.sec, msg.header.stamp.nanosec) for msg in raw}
            self.assertTrue(any((msg.header.stamp.sec, msg.header.stamp.nanosec) in raw_stamps
                                for msg in public))

        with self.subTest('loss with ongoing positive target cannot accelerate on fake zero'):
            self.run_for(0.6, speed=9.5, target=10.0)  # Build nonzero PID state.
            self.assertGreater(self.commands[-1][1][5], 0.3)
            start = self.run_for(0.7, target=10.0)
            self.assert_zero_fallback(start)

        with self.subTest('recovery resets integral and derivative without a pedal spike'):
            start = self.run_for(0.6, speed=10.0, target=10.0)
            recovered = [cmd for t, cmd in self.commands if t > start]
            self.assertGreater(len(recovered), 3)
            for cmd in recovered:
                self.assertAlmostEqual(cmd[5], 0.0, places=5)
                self.assertAlmostEqual(cmd[6], 0.0, places=5)

        with self.subTest('watchdog transmits even when control_cmd also stops'):
            self.run_for(0.3, speed=9.5, target=10.0)
            self.assertGreater(self.commands[-1][1][5], 0.3)
            start = self.run_for(0.7)
            self.assert_zero_fallback(start)

        with self.subTest('real standstill still applies standstill brake'):
            start = self.run_for(0.5, speed=0.0, target=0.0)
            commands = [cmd for t, cmd in self.commands if t > start + 0.3]
            self.assertGreater(len(commands), 3)
            for cmd in commands:
                self.assertEqual(cmd[5], 0.0)
                self.assertAlmostEqual(cmd[6], 0.1, places=6)

        with self.subTest('brake remains capped at 0.63'):
            start = self.run_for(0.5, speed=15.0, target=0.0)
            commands = [cmd for t, cmd in self.commands if t > start + 0.3]
            self.assertGreater(len(commands), 3)
            for cmd in commands:
                self.assertEqual(cmd[5], 0.0)
                self.assertAlmostEqual(cmd[6], 0.63, places=6)

        for bad_speed in (float('nan'), float('inf')):
            with self.subTest('nonfinite UDP cannot keep feedback alive', speed=bad_speed):
                self.run_for(0.3, speed=10.0, target=10.0)
                start = self.run_for(0.7, speed=bad_speed, target=10.0)
                self.assert_zero_fallback(start)

        with self.subTest('malformed UDP cannot keep feedback alive'):
            self.run_for(0.3, speed=10.0, target=10.0)
            start = self.run_for(0.7, target=10.0, malformed=True)
            self.assert_zero_fallback(start)

        with self.subTest('normal IMU is not replaced'):
            start = self.run_for(0.5, speed=10.0, target=10.0, imu=True)
            self.assert_real_imu(start)

        with self.subTest('joint loss sustains zero IMU and pedals without refreshing raw input'):
            start = self.run_for(0.9, target=10.0)
            self.assert_zero_fallback(start)
            self.assert_imu_fallback(start)

        with self.subTest('IMU recovery immediately restores real samples even if vehicle is stale'):
            start = self.run_for(0.5, target=10.0, imu=True)
            self.assert_real_imu(start)
            self.assert_zero_fallback(start)

        with self.subTest('IMU loss alone must not synthesize a stopped gyro'):
            self.run_for(0.3, speed=10.0, target=10.0, imu=True)
            start = self.run_for(0.6, speed=10.0, target=10.0)
            self.assertFalse([msg for t, msg in self.imu_samples if t > start + 0.3])

        with self.subTest('vehicle recovery disables synthetic IMU before IMU resumes'):
            start = self.run_for(0.6, target=10.0)
            self.assert_imu_fallback(start)
            start = self.run_for(0.5, speed=10.0, target=10.0)
            self.assertFalse([msg for t, msg in self.imu_samples if t > start + 0.1])

        with self.subTest('vehicle loss alone must not overwrite real IMU'):
            self.run_for(0.3, speed=10.0, target=10.0, imu=True)
            start = self.run_for(0.6, target=10.0, imu=True)
            self.assert_real_imu(start)
            self.assert_zero_fallback(start)

        for invalid in (b'bad', imu_packet((math.nan, 0.0, 0.0)),
                        imu_packet((0.0, math.inf, 0.0))):
            with self.subTest('invalid IMU cannot refresh timeout', payload=repr(invalid[:12])):
                self.run_for(0.3, speed=10.0, target=10.0, imu=True)
                start = self.run_for(0.7, target=10.0, imu_payload=invalid)
                self.assert_imu_fallback(start)
                self.assert_zero_fallback(start)

        with self.subTest('duplicate IMU timestamp is not fresh data'):
            self.run_for(0.3, speed=10.0, target=10.0, imu=True)
            start = self.run_for(0.7, target=10.0, imu_payload=self.last_sent_imu)
            self.assert_imu_fallback(start)

        with self.subTest('full recovery has no trailing synthetic IMU or pedal spike'):
            start = self.run_for(0.5, speed=10.0, target=10.0, imu=True)
            self.assert_real_imu(start)
            commands = [cmd for t, cmd in self.commands if t > start + 0.1]
            self.assertGreater(len(commands), 3)
            for cmd in commands:
                self.assertEqual(cmd[5:7], (0.0, 0.0))

    def test_timeout_parameter_validation(self):
        for name in ('imu_timeout_sec', 'velocity_timeout_sec'):
            for value in ('0.0', '-0.1', '.nan', '.inf'):
                with self.subTest(parameter=name, value=value):
                    result = subprocess.run(
                        [sys.executable, EXECUTABLES.sensor, '--ros-args',
                         '-p', 'bind_ip:=127.0.0.1', '-p', f'{name}:={value}'],
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                        text=True, timeout=5)
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn(f'{name} must be finite and positive', result.stdout)


if __name__ == '__main__':
    # Select the middleware before importing rclpy (which loads the RMW library).
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    for key in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE'):
        os.environ.pop(key, None)
    import rclpy

    parser = argparse.ArgumentParser()
    parser.add_argument('--receiver', required=True)
    parser.add_argument('--sender', required=True)
    parser.add_argument('--sensor', required=True)
    EXECUTABLES, remaining = parser.parse_known_args()
    unittest.main(argv=[__file__, *remaining], verbosity=2)
