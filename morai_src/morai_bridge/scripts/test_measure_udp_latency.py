#!/usr/bin/env python3
"""Offline parser and timing regression check: python3 test_measure_udp_latency.py."""
import csv
import json
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile

from measure_udp_latency import analyze, packet_stamp, pcap_packets, udp_header


def ethernet(payload, source="192.168.0.27", destination="192.168.0.37", port=9001):
    udp = struct.pack("!HHHH", 50000, port, len(payload) + 8, 0) + payload
    ip = struct.pack("!BBHHHBBH4s4s", 0x45, 0, 20 + len(udp), 0, 0, 64, 17, 0,
                     socket.inet_aton(source), socket.inet_aton(destination))
    return bytes(12) + b"\x08\x00" + ip + udp


def main():
    stamp = 1_700_000_000
    mor = b"MOR" + struct.pack("<IIII", stamp, 0, 0, 4) + b"jpegEI"
    chunk = b"MOR" + struct.pack("<IIII", stamp, 0, 1, 4) + b"jpegEI"
    next_frame = b"MOR" + struct.pack("<IIII", stamp, 50_000_000, 0, 4) + b"jpegEI"
    frame = ethernet(mor)
    assert packet_stamp(mor, len(mor)) == ("MOR", stamp * 10**9)
    assert packet_stamp(b"MOR" + struct.pack("<IIII", stamp, 10**9, 0, 0), 21)[1] is None
    assert packet_stamp(mor, 21)[1] is None  # advertised payload does not fit
    status = b"#MoraiInfo$" + struct.pack("<I", 152) + bytes(12) + struct.pack("<II", stamp, 42) + bytes(146)
    assert len(status) == 181
    assert packet_stamp(status, 181)[1] == stamp * 10**9 + 42
    assert packet_stamp(status, 229)[1] is None  # unknown version must not invent age
    for magic, offset, length in ((b"#IMUData$", 25, 115), (b"#CollisionData$", 31, 181)):
        data = magic + bytes(offset - len(magic)) + struct.pack("<II", stamp, 99)
        assert packet_stamp(data, length)[1] == stamp * 10**9 + 99
    assert udp_header(1, frame)[-1] == mor
    vlan = frame[:12] + b"\x81\x00\x00\x01" + frame[12:]
    assert udp_header(1, vlan)[-1] == mor
    assert udp_header(113, bytes(14) + b"\x08\x00" + frame[14:])[-1] == mor
    assert udp_header(276, b"\x08\x00" + bytes(18) + frame[14:])[-1] == mor
    fragmented = bytearray(frame)
    struct.pack_into("!H", fragmented, 20, 0x2000)
    assert udp_header(1, fragmented) is not None  # first IP fragment
    struct.pack_into("!H", fragmented, 20, 1)
    assert udp_header(1, fragmented) is None  # later IP fragment has no UDP header
    assert udp_header(1, frame[:30]) is None
    with tempfile.TemporaryDirectory() as directory:
        output = Path(directory)
        pcap = output / "input.pcap"
        with pcap.open("wb") as stream:
            stream.write(struct.pack("<IHHIIII", 0xA1B2C3D4, 2, 4, 0, 0, 128, 1))
            for usec, packet in ((10_000, frame), (11_000, ethernet(chunk)),
                                 (70_000, ethernet(next_frame)),
                                 (75_000, ethernet(b"#MoraiCtrlCmd$", "192.168.0.37", "192.168.0.27", 9091))):
                stream.write(struct.pack("<IIII", stamp, usec, len(packet), len(packet)))
                stream.write(packet)
        result = analyze(pcap, output, "192.168.0.27", clock_offset_ms=5, gap_ms=55)
        camera = result["RX:9001:MOR"]
        assert camera["observed_datagrams"] == 3 and camera["samples"] == 2
        assert camera["interval_ms"]["max"] == 60
        assert camera["source_interval_ms"]["max"] == 50
        assert camera["raw_age_ms"]["mean"] == 15
        assert camera["corrected_age_ms"]["mean"] == 10
        assert camera["gaps_over_threshold"] == 1
        assert result["TX:9091:CTRL"]["raw_age_ms"] is None
        with (output / "samples.csv").open() as stream:
            assert len(list(csv.DictReader(stream))) == 3
        subprocess.run([sys.executable, str(Path(__file__).with_name("measure_udp_latency.py")),
                        "--pcap", str(pcap), "--output", str(output / "reports"),
                        "--label", "offline", "--clock-offset-ms", "5"], check=True)
        report = json.loads(next((output / "reports").glob("*/summary.json")).read_text())
        assert report["streams"]["RX:9001:MOR"]["samples"] == 2
        pcap.write_bytes(pcap.read_bytes()[:-1])
        try:
            list(pcap_packets(pcap))
        except ValueError:
            pass
        else:
            raise AssertionError("Truncated pcap accepted")
    print("PASS: timestamps, bounds, Ethernet/VLAN/SLL, fragments, frame dedup, TX, timing, truncated pcap")


if __name__ == "__main__":
    main()
