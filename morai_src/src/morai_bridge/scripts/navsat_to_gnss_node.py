#!/usr/bin/python3

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from pyproj import Transformer
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus


def utm_epsg(latitude, longitude, zone=0):
    zone = zone or int((longitude + 180.0) // 6.0) + 1
    if not 1 <= zone <= 60:
        raise ValueError(f"invalid UTM zone: {zone}")
    return (32600 if latitude >= 0.0 else 32700) + zone


assert utm_epsg(37.238745, 126.77282) == 32652


class NavSatToGnss(Node):
    def __init__(self):
        super().__init__("navsat_to_gnss_node")

        self.declare_parameter("input_topic", "/gps/fix")
        self.declare_parameter("output_topic", "/gnss")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("utm_zone", 0)

        self.utm_zone = self.get_parameter("utm_zone").value
        self.transformer = None
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

        if self.transformer is None:
            epsg = utm_epsg(fix.latitude, fix.longitude, self.utm_zone)
            self.transformer = Transformer.from_crs(4326, epsg, always_xy=True)
            self.get_logger().info(
                f"GNSS UTM projection: EPSG:{epsg}"
            )

        x, y = self.transformer.transform(fix.longitude, fix.latitude)
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
