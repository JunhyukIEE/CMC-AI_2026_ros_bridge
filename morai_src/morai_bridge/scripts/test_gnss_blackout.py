#!/usr/bin/env python3
"""Offline regression check after sourcing ROS and the morai_msgs workspace.

Run: python3 test_gnss_blackout.py. No ROS node or UDP socket is started.
"""
import math
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from sensor_msgs.msg import NavSatStatus

from morai_sensor_receiver_node import MoraiSensorReceiver


def main():
    messages = []
    clock = SimpleNamespace(now=lambda: SimpleNamespace(
        to_msg=lambda: Time(sec=len(messages) + 1, nanosec=123)))
    receiver = SimpleNamespace(
        gnss_frame="gnss_link", last_altitude=0.0,
        get_clock=lambda: clock,
        gnss_pub=SimpleNamespace(publish=messages.append))

    def publish(lat="3730.0", lon="12715.0", alt="42.0", quality="1", ns="N", ew="E"):
        sentence = f"$GPGGA,120000,{lat},{ns},{lon},{ew},{quality},10,1.0,{alt},M,0,M,,\r\n"
        count = len(messages)
        MoraiSensorReceiver._publish_gnss(receiver, sentence.encode("ascii"))
        assert len(messages) == count + 1, "Invalid fix must still be published"
        msg = messages[-1]
        assert msg.header.frame_id == "gnss_link"
        assert msg.header.stamp == Time(sec=count + 1, nanosec=123)
        assert msg.status.service == NavSatStatus.SERVICE_GPS
        return msg

    fix, no_fix = NavSatStatus.STATUS_FIX, NavSatStatus.STATUS_NO_FIX
    normal = publish()
    blackout = publish(lat="0", lon="0", alt="0")
    recovered = publish(lat="3745.0", lon="12730.0", alt="43.0")
    assert [m.status.status for m in messages] == [fix, no_fix, fix]
    assert (normal.latitude, normal.longitude, normal.altitude) == (37.5, 127.25, 42.0)
    assert (blackout.latitude, blackout.longitude, blackout.altitude) == (0.0, 0.0, 0.0)
    assert (recovered.latitude, recovered.longitude, recovered.altitude) == (37.75, 127.5, 43.0)

    for quality in ("0", ""):
        assert publish(quality=quality).status.status == no_fix
    for quality in ("1", "2", "4", "5"):
        assert publish(quality=quality).status.status == fix
    for coords in ({"lat": "0"}, {"lon": "0"},
                   {"lat": "9000", "lon": "18000"},
                   {"lat": "9000", "lon": "18000", "ns": "S", "ew": "W"}):
        assert publish(**coords).status.status == fix
    assert publish(lat="0", lon="0", alt="50").status.status == no_fix
    for coords in ({"lat": "9000.01"}, {"lat": "9000.01", "ns": "S"},
                   {"lon": "18000.01"}, {"lon": "18000.01", "ew": "W"}):
        assert publish(**coords).status.status == no_fix
    for field, attr in (("lat", "latitude"), ("lon", "longitude"), ("alt", "altitude")):
        for value in ("nan", "inf", "-inf"):
            msg = publish(**{field: value})
            assert msg.status.status == no_fix
            assert not math.isfinite(getattr(msg, attr))
    assert publish().status.status == fix

    count = len(messages)
    MoraiSensorReceiver._publish_gnss(receiver, b"$GPRMC,ignored\r\n")
    assert len(messages) == count, "Only GPGGA should publish fixes"
    print(f"PASS: {count} GPGGA cases, recovery, unchanged headers, and GPRMC ignored")


if __name__ == "__main__":
    main()
