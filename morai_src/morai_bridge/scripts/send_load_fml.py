#!/usr/bin/env python3

import argparse
import ipaddress
import socket


DEFAULT_SIMULATOR_IP = "192.168.0.27"
RESET_PORT = 5005
RESET_MESSAGE = b"LOAD_FML"


def main():
    parser = argparse.ArgumentParser(description="Send the MORAI LOAD_FML reset message.")
    parser.add_argument(
        "--ip",
        type=ipaddress.IPv4Address,
        default=ipaddress.IPv4Address(DEFAULT_SIMULATOR_IP),
        help=f"MORAI simulator IPv4 address (default: {DEFAULT_SIMULATOR_IP})",
    )
    args = parser.parse_args()
    destination = (str(args.ip), RESET_PORT)

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sent = sock.sendto(RESET_MESSAGE, destination)

    if sent != len(RESET_MESSAGE):
        raise RuntimeError(f"Sent {sent} of {len(RESET_MESSAGE)} bytes")
    print(f"Sent LOAD_FML to {destination[0]}:{destination[1]}")


if __name__ == "__main__":
    main()
