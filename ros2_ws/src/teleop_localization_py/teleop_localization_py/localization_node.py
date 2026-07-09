#!/usr/bin/env python3
"""Localization + velocity publisher (Python).

Simulates an RR100-style ground robot driving near the OvGU Magdeburg campus.
A unicycle motion model is integrated from a *clamped* Twist command
(|linear| <= 0.2 m/s, |angular| <= 0.2 rad/s), and the resulting local
ENU pose is converted to WGS-84 latitude/longitude using an equirectangular
approximation around a fixed origin.

Published topics
----------------
  /robot/gps   (sensor_msgs/NavSatFix)  -- the localization point for the map
  /cmd_vel     (geometry_msgs/Twist)    -- the commanded body velocity

Everything runs at a single fixed rate (default 30 Hz).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import Twist

# WGS-84 mean Earth radius (metres). Good enough for campus-scale dead reckoning.
EARTH_RADIUS_M = 6_378_137.0


def clamp(value: float, limit: float) -> float:
    """Symmetric clamp to [-limit, +limit]."""
    return max(-limit, min(limit, value))


class LocalizationNode(Node):
    def __init__(self) -> None:
        super().__init__("localization_node")

        # ---- Parameters (override from launch / CLI) ---------------------
        self.declare_parameter("center_lat", 52.139200)   # OvGU Universitaetsplatz
        self.declare_parameter("center_lon", 11.645200)
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("max_linear", 0.2)          # m/s   (hard limit)
        self.declare_parameter("max_angular", 0.2)         # rad/s (hard limit)

        self.lat0 = self.get_parameter("center_lat").value
        self.lon0 = self.get_parameter("center_lon").value
        self.rate = float(self.get_parameter("rate_hz").value)
        self.max_lin = float(self.get_parameter("max_linear").value)
        self.max_ang = float(self.get_parameter("max_angular").value)
        self.dt = 1.0 / self.rate

        # Pre-compute the metres-per-degree scale at the origin latitude.
        self._m_per_deg_lat = (math.pi / 180.0) * EARTH_RADIUS_M
        self._m_per_deg_lon = self._m_per_deg_lat * math.cos(math.radians(self.lat0))

        # ---- Pose state in local ENU frame (metres / radians) ------------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.t = 0.0  # elapsed seconds, drives the demo trajectory

        # Sensor data is best-effort; the bridge only cares about the latest.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.gps_pub = self.create_publisher(NavSatFix, "/robot/gps", sensor_qos)
        self.twist_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.timer = self.create_timer(self.dt, self.on_tick)
        self.get_logger().info(
            f"localization_node up @ {self.rate:.0f} Hz, origin "
            f"({self.lat0:.6f}, {self.lon0:.6f}), "
            f"limits lin<= {self.max_lin} m/s ang<= {self.max_ang} rad/s"
        )

    # ---------------------------------------------------------------------
    def demo_command(self) -> tuple[float, float]:
        """Return a (linear, angular) command for the demo trajectory.

        Drive forward at the maximum allowed speed while slowly weaving so the
        marker traces a visible arc on the map. Replace this with a real
        /cmd_vel subscriber to drive from a gamepad or planner.
        """
        #figure-8 weaving motion
        # linear = self.max_lin
        # angular = self.max_ang * math.sin(0.10 * self.t)

        #tight-circle weaving motion (with max turn angle)
        # linear = self.max_lin
        # angular = self.max_ang

        #straigt line motion
        linear = self.max_lin
        angular = 0.0

        #stop scenario i.e. no motion
        # linear = 0.0
        # angular = 0.0

        return linear, angular

    def on_tick(self) -> None:
        self.t += self.dt
        lin_cmd, ang_cmd = self.demo_command()

        # Enforce the hard velocity envelope regardless of the command source.
        v = clamp(lin_cmd, self.max_lin)
        w = clamp(ang_cmd, self.max_ang)

        # Unicycle integration (forward Euler is fine at 30 Hz / 0.2 m/s).
        self.theta += w * self.dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        self.x += v * math.cos(self.theta) * self.dt
        self.y += v * math.sin(self.theta) * self.dt

        # Local ENU offset -> WGS-84.
        lat = self.lat0 + self.y / self._m_per_deg_lat
        lon = self.lon0 + self.x / self._m_per_deg_lon

        now = self.get_clock().now().to_msg()

        fix = NavSatFix()
        fix.header.stamp = now
        fix.header.frame_id = "map"
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = lat
        fix.longitude = lon
        fix.altitude = 55.0  # ~Magdeburg elevation
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.gps_pub.publish(fix)

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.twist_pub.publish(twist)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationNode()
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
