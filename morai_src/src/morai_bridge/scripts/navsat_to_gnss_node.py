#!/usr/bin/python3

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus


EARTH_RADIUS_M = 6378137.0


def local_xy(latitude, longitude, origin_latitude, origin_longitude):
    origin_latitude_rad = math.radians(origin_latitude)
    x = EARTH_RADIUS_M * math.cos(origin_latitude_rad) * math.radians(
        longitude - origin_longitude
    )
    y = EARTH_RADIUS_M * math.radians(latitude - origin_latitude)
    return x, y


assert local_xy(35.0, 126.0, 35.0, 126.0) == (0.0, 0.0)


class NavSatToGnss(Node):
    def __init__(self):
        super().__init__("navsat_to_gnss_node")

        self.declare_parameter("input_topic", "/gps/fix")
        self.declare_parameter("output_topic", "/gnss")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("origin_latitude", 0.0)
        self.declare_parameter("origin_longitude", 0.0)

        origin_latitude = self.get_parameter("origin_latitude").value
        origin_longitude = self.get_parameter("origin_longitude").value
        self.origin = (
            None
            if origin_latitude == 0.0 and origin_longitude == 0.0
            else (origin_latitude, origin_longitude)
        )
        self.frame_id = self.get_parameter("frame_id").value

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter("output_topic").value,
            10,
        )
        self.subscription = self.create_subscription(
            NavSatFix,
            self.get_parameter("input_topic").value,
            self.convert,
            qos_profile_sensor_data,
        )

    def convert(self, fix):
        if fix.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        if not math.isfinite(fix.latitude) or not math.isfinite(fix.longitude):
            return

        if self.origin is None:
            self.origin = (fix.latitude, fix.longitude)
            self.get_logger().info(
                f"GNSS ENU origin: lat={fix.latitude:.9f}, lon={fix.longitude:.9f}"
            )

        x, y = local_xy(fix.latitude, fix.longitude, *self.origin)
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = fix.header.stamp
        pose.header.frame_id = self.frame_id
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.position.z = fix.altitude if math.isfinite(fix.altitude) else 0.0
        pose.pose.pose.orientation.w = 1.0
        self.publisher.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = NavSatToGnss()
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
