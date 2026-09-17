#!/usr/bin/env python3
"""Passive MORAI IPv4 UDP timing capture. No ROS imports, binds or control sends.

See measure_udp_latency.md. Capture needs tcpdump privileges; --pcap does not.
"""

import argparse
import collections
import csv
import datetime
import ipaddress
import json
import math
from pathlib import Path
import re
import signal
import statistics
import struct
import subprocess
import time


def pcap_packets(path):
    """Read classic pcap (tcpdump -w), including micro/nanosecond variants."""
    with open(path, "rb") as stream:
        header = stream.read(24)
        formats = {b"\xd4\xc3\xb2\xa1": ("<", 1000),
                   b"\xa1\xb2\xc3\xd4": (">", 1000),
                   b"\x4d\x3c\xb2\xa1": ("<", 1),
                   b"\xa1\xb2\x3c\x4d": (">", 1)}
        if len(header) != 24 or header[:4] not in formats:
            raise ValueError("Expected classic pcap, not pcapng; use tcpdump -w")
        endian, scale = formats[header[:4]]
        linktype = struct.unpack_from(endian + "I", header, 20)[0]
        if linktype not in (1, 113, 276):
            raise ValueError(f"Unsupported pcap link type {linktype}")
        while True:
            record = stream.read(16)
            if not record:
                return
            if len(record) != 16:
                raise ValueError("Truncated pcap record")
            sec, fraction, size, _ = struct.unpack(endian + "IIII", record)
            if size > 1_048_576:
                raise ValueError("Invalid captured packet length")
            packet = stream.read(size)
            if len(packet) != size:
                raise ValueError("Truncated captured packet")
            yield sec * 1_000_000_000 + fraction * scale, linktype, packet


def udp_header(linktype, packet):
    """Parse first IPv4 fragment only; captured payload may be snaplen-truncated."""
    if linktype == 1:  # Ethernet, with optional VLAN tags
        if len(packet) < 14:
            return None
        protocol, offset = struct.unpack_from("!H", packet, 12)[0], 14
        while protocol in (0x8100, 0x88A8):
            if len(packet) < offset + 4:
                return None
            protocol = struct.unpack_from("!H", packet, offset + 2)[0]
            offset += 4
    else:  # Linux cooked v1/v2, including tcpdump -i any
        offset = 16 if linktype == 113 else 20
        if len(packet) < offset:
            return None
        protocol = struct.unpack_from("!H", packet, 14 if linktype == 113 else 0)[0]
    if protocol != 0x0800 or len(packet) < offset + 20:
        return None
    ip = packet[offset:]
    ihl = (ip[0] & 15) * 4
    total = struct.unpack_from("!H", ip, 2)[0]
    if (ip[0] >> 4 != 4 or ihl < 20 or len(ip) < ihl + 8 or
            total < ihl + 8 or ip[9] != 17 or
            struct.unpack_from("!H", ip, 6)[0] & 0x1FFF):
        return None
    src, dst = str(ipaddress.IPv4Address(bytes(ip[12:16]))), str(ipaddress.IPv4Address(bytes(ip[16:20])))
    sport, dport, size = struct.unpack_from("!HHH", ip, ihl)
    if size < 8:
        return None
    return src, dst, sport, dport, size - 8, ip[ihl + 8:min(total, ihl + size)]


def packet_stamp(payload, udp_size):
    """Known local bridge layouts only. Unknown protocols retain cadence metrics."""
    kind, offset = "UDP", None
    if payload[:3] in (b"MOR", b"BOX"):
        kind = payload[:3].decode()
        if len(payload) >= 19 and udp_size >= 21:
            size = struct.unpack_from("<I", payload, 15)[0]
            if size <= udp_size - 21:
                offset = 3
    elif payload.startswith(b"#MoraiInfo$"):
        kind = "STATUS"
        if udp_size == 181 and len(payload) >= 35 and struct.unpack_from("<I", payload, 11)[0] == 152:
            offset = 27
    elif payload.startswith(b"#IMUData$"):
        kind = "IMU"
        if udp_size == 115:
            offset = 25
    elif payload.startswith(b"#CollisionData$"):
        kind = "COLLISION"
        if udp_size == 181:
            offset = 31
    elif payload.startswith(b"#MoraiCtrlCmd$"):
        kind = "CTRL"
    elif payload.startswith((b"$G", b"$P")):
        kind = "NMEA"  # UTC time-of-day alone is not a full epoch timestamp.
    if offset is not None and len(payload) >= offset + 8:
        sec, nsec = struct.unpack_from("<II", payload, offset)
        if nsec < 1_000_000_000 and (sec or nsec):
            return kind, sec * 1_000_000_000 + nsec
    return kind, None


def stats(values):
    if not values:
        return None
    ordered = sorted(values)
    return {"n": len(values), "min": ordered[0], "mean": statistics.fmean(values),
            "p50": ordered[math.ceil(len(values) * .5) - 1],
            "p95": ordered[math.ceil(len(values) * .95) - 1],
            "p99": ordered[math.ceil(len(values) * .99) - 1], "max": ordered[-1]}


def analyze(pcap, output, sim_ip, clock_offset_ms=None, gap_ms=200):
    groups = {}
    fields = ["capture_ns", "direction", "port", "protocol", "udp_payload_bytes",
              "source_stamp_ns", "raw_age_ms", "corrected_age_ms", "interval_ms",
              "source_interval_ms"]
    with (output / "samples.csv").open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for arrival, linktype, packet in pcap_packets(pcap):
            parsed = udp_header(linktype, packet)
            if parsed is None:
                continue
            src, dst, sport, dport, size, payload = parsed
            if src == sim_ip:
                direction, port = "RX", dport
            elif dst == sim_ip:
                direction, port = "TX", dport
            else:
                continue
            kind, stamp = packet_stamp(payload, size)
            key = f"{direction}:{port}:{kind}"
            g = groups.setdefault(key, {"packets": 0, "samples": 0, "seen": collections.deque(maxlen=512),
                                       "interval": [], "age": [], "source_interval": [],
                                       "last": None, "last_stamp": None, "first": arrival,
                                       "backwards_capture": 0, "backwards_stamp": 0})
            g["packets"] += 1
            # Camera chunks share a timestamp. Report first observed chunk per frame,
            # NOT frame completion/decode time. Bounded history also handles reordering.
            if kind in ("MOR", "BOX") and stamp is not None:
                if stamp in g["seen"]:
                    continue
                g["seen"].append(stamp)
            interval = None if g["last"] is None else (arrival - g["last"]) / 1e6
            source_interval = None if stamp is None or g["last_stamp"] is None else (stamp - g["last_stamp"]) / 1e6
            age = None if stamp is None else (arrival - stamp) / 1e6
            for name, value in (("interval", interval), ("source_interval", source_interval), ("age", age)):
                if value is not None:
                    g[name].append(value)
            g["backwards_capture"] += int(interval is not None and interval < 0)
            g["backwards_stamp"] += int(source_interval is not None and source_interval < 0)
            g["last"], g["last_stamp"] = arrival, stamp
            g["samples"] += 1
            writer.writerow(dict(zip(fields, [arrival, direction, port, kind, size, stamp, age,
                None if age is None or clock_offset_ms is None else age - clock_offset_ms,
                interval, source_interval])))
    result = {}
    for key, g in groups.items():
        span = (g["last"] - g["first"]) / 1e9
        result[key] = {"observed_datagrams": g["packets"], "samples": g["samples"],
                       "sample_hz": (g["samples"] - 1) / span if span > 0 else None,
                       "interval_ms": stats(g["interval"]), "source_interval_ms": stats(g["source_interval"]),
                       "raw_age_ms": stats(g["age"]),
                       "corrected_age_ms": stats([v - clock_offset_ms for v in g["age"]]) if clock_offset_ms is not None else None,
                       "gaps_over_threshold": sum(v > gap_ms for v in g["interval"]),
                       "capture_time_backwards": g["backwards_capture"],
                       "source_time_backwards": g["backwards_stamp"]}
    return result


def capture(args, output):
    # Snaplen preserves headers only, keeping camera recording overhead modest.
    command = ["tcpdump", "-i", args.interface, "-n", "-p", "-s", "128", "-B", "4096",
               "-U", "-w", "-", "host", args.sim_ip, "and", "udp"]
    ping = None
    with (output / "capture.log").open("x") as log, (output / "ping.log").open("x") as ping_log, \
            (output / "headers.pcap").open("xb") as pcap_stream:
        # Pre-open the file: tcpdump may drop privileges after starting under sudo.
        process = subprocess.Popen(command, stdout=pcap_stream, stderr=log)
        try:
            if not args.no_ping:
                ping = subprocess.Popen(["ping", "-n", "-i", "1", "-W", "1", "-c",
                                         str(math.ceil(args.duration)), args.sim_ip],
                                        stdout=ping_log, stderr=subprocess.STDOUT, env={"LC_ALL": "C"})
            try:
                process.wait(timeout=args.duration)
            except subprocess.TimeoutExpired:
                pass
            except KeyboardInterrupt:
                print("\nStopping capture and saving results...")
        finally:
            for child in (process, ping):
                if child is not None and child.poll() is None:
                    child.send_signal(signal.SIGINT)
                    try:
                        child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        child.kill()
                        child.wait()
    if process.returncode != 0:
        raise RuntimeError(f"tcpdump failed; see {output / 'capture.log'}. Capture requires sudo/CAP_NET_RAW.")
    return output / "headers.pcap"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sim-ip", default="192.168.0.1")
    parser.add_argument("--interface", default="enp8s0", help="Prefer one physical interface; any may duplicate traffic")
    parser.add_argument("--duration", type=float, default=60)
    parser.add_argument("--label", default="baseline")
    parser.add_argument("--output", type=Path, default=Path("udp_latency_results"))
    parser.add_argument("--pcap", type=Path, help="Analyze an existing classic pcap without root or network activity")
    parser.add_argument("--no-ping", action="store_true", help="Disable the 1 Hz ICMP RTT probe")
    parser.add_argument("--clock-offset-ms", type=float, help="Measured bridge clock minus simulator clock; never estimated from minimum age")
    parser.add_argument("--gap-ms", type=float, default=200, help="Count inter-sample gaps above this value")
    args = parser.parse_args()
    args.sim_ip = str(ipaddress.IPv4Address(args.sim_ip))
    for name in ("duration", "gap_ms"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(f"{name} must be finite and positive")
    if args.clock_offset_ms is not None and not math.isfinite(args.clock_offset_ms):
        parser.error("clock-offset-ms must be finite")
    if not re.fullmatch(r"[A-Za-z0-9_-]+", args.label):
        parser.error("label: use letters, numbers, underscores or hyphens")
    output = args.output / (datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f_") + args.label)
    output.mkdir(parents=True, exist_ok=False)
    print(f"Results: {output.resolve()}", flush=True)
    start = time.monotonic()
    try:
        pcap = args.pcap if args.pcap else capture(args, output)
        streams = analyze(pcap, output, args.sim_ip, args.clock_offset_ms, args.gap_ms)
        capture_log = (output / "capture.log").read_text() if not args.pcap else "External pcap: capture drops unknown"
        ping_log = (output / "ping.log").read_text() if not args.pcap else ""
        rtts = [float(v) for v in re.findall(r"time=([0-9.]+) ms", ping_log)]
        report = {"sim_ip": args.sim_ip, "interface": args.interface, "label": args.label,
                  "pcap": str(pcap.resolve()), "elapsed_sec": time.monotonic() - start,
                  "clock_offset_ms": args.clock_offset_ms, "gap_threshold_ms": args.gap_ms,
                  "capture_log": capture_log, "ping_log": ping_log, "ping_rtt_ms": stats(rtts),
                  "limitations": ["Raw age includes clock offset and simulator pipeline; not pure network latency.",
                                  "Simulation-relative timestamps cannot be compared to wall clock without a valid mapping.",
                                  "Camera sample is first observed fragment/chunk, not completed frame or ROS arrival.",
                                  "TX measures host send cadence only; remote delivery/application is unobserved.",
                                  "Capture loss, host clock steps and idle/event streams affect gap counts; these are not packet-loss estimates."],
                  "streams": streams}
        (output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
        print("stream                  samples      Hz   gap_p95_ms gap_max_ms  raw_age_p95_ms")
        for key, s in streams.items():
            def value(metric, field):
                return f"{s[metric][field]:.3f}" if s[metric] else "n/a"
            hz = f"{s['sample_hz']:.2f}" if s["sample_hz"] is not None else "n/a"
            print(f"{key:24} {s['samples']:7} {hz:>7} {value('interval_ms', 'p95'):>12} "
                  f"{value('interval_ms', 'max'):>10} {value('raw_age_ms', 'p95'):>15}")
        print("Ping RTT:", stats(rtts))
        print(capture_log.strip())
        print("raw_age is NOT verified one-way delay. See summary.json limitations and clock offset.")
        if not streams:
            print("No matching UDP observed: verify simulator IP, interface and active streams.")
    except (OSError, ValueError, RuntimeError) as error:
        parser.exit(1, f"Error: {error}\n")


if __name__ == "__main__":
    main()
