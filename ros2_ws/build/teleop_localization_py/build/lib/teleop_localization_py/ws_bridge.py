#!/usr/bin/env python3
"""ROS 2 -> WebSocket bridge.

Subscribes to the localization point and the velocity command, keeps only the
most recent sample of each, and pushes a combined JSON frame to every connected
browser at a fixed rate (default 30 Hz). The browser never talks ROS; it just
receives small JSON frames over a plain WebSocket.

Frame schema (one line of JSON per tick):
    {
      "type": "state",
      "seq":   <int>,            # monotonically increasing frame counter
      "t":     <float seconds>,  # bridge wall-clock timestamp
      "lat":   <float>,          # degrees, WGS-84
      "lon":   <float>,
      "fix":   <int>,            # NavSatStatus.status
      "linear":  <float m/s>,
      "angular": <float rad/s>
    }

Design note: rclpy spins in a background thread; the asyncio WebSocket server
owns the main thread. The two share a single dict guarded by a lock.
"""

import asyncio
import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist

import websockets


class BridgeNode(Node):
    """ROS side: collect the latest GPS fix and Twist into a shared dict."""

    def __init__(self, shared_state: dict, lock: threading.Lock) -> None:
        super().__init__("ws_bridge")
        self._state = shared_state
        self._lock = lock

        self.declare_parameter("gps_topic", "/robot/gps")
        self.declare_parameter("twist_topic", "/cmd_vel")
        gps_topic = self.get_parameter("gps_topic").value
        twist_topic = self.get_parameter("twist_topic").value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(NavSatFix, gps_topic, self._on_gps, sensor_qos)
        self.create_subscription(Twist, twist_topic, self._on_twist, 10)
        self.get_logger().info(
            f"ws_bridge subscribed: {gps_topic} (NavSatFix), {twist_topic} (Twist)"
        )

    def _on_gps(self, msg: NavSatFix) -> None:
        with self._lock:
            self._state["lat"] = msg.latitude
            self._state["lon"] = msg.longitude
            self._state["fix"] = int(msg.status.status)

    def _on_twist(self, msg: Twist) -> None:
        with self._lock:
            self._state["linear"] = msg.linear.x
            self._state["angular"] = msg.angular.z


# --------------------------------------------------------------------------- #
#  WebSocket server (asyncio main thread)
# --------------------------------------------------------------------------- #
class WsServer:
    def __init__(self, host: str, port: int, rate_hz: float,
                 shared_state: dict, lock: threading.Lock) -> None:
        self.host = host
        self.port = port
        self.period = 1.0 / rate_hz
        self.state = shared_state
        self.lock = lock
        self.clients: set = set()
        self._seq = 0

    async def _handler(self, websocket):
        """Register a client for the lifetime of its connection."""
        self.clients.add(websocket)
        try:
            await websocket.wait_closed()
        finally:
            self.clients.discard(websocket)

    async def _broadcast_loop(self):
        """Emit one combined state frame to all clients at the fixed rate."""
        while True:
            tick = asyncio.get_event_loop().time()
            if self.clients:
                with self.lock:
                    snapshot = dict(self.state)
                self._seq += 1
                frame = json.dumps({
                    "type": "state",
                    "seq": self._seq,
                    "t": time.time(),
                    "lat": snapshot.get("lat"),
                    "lon": snapshot.get("lon"),
                    "fix": snapshot.get("fix", 0),
                    "linear": snapshot.get("linear", 0.0),
                    "angular": snapshot.get("angular", 0.0),
                })
                # websockets.broadcast fans out without awaiting each client.
                websockets.broadcast(self.clients, frame)
            # Keep a steady cadence even if a tick ran long.
            await asyncio.sleep(max(0.0, self.period - (asyncio.get_event_loop().time() - tick)))

    async def run(self):
        async with websockets.serve(self._handler, self.host, self.port):
            print(f"[ws_bridge] serving ws://{self.host}:{self.port} @ {1.0 / self.period:.0f} Hz")
            await self._broadcast_loop()


def _spin_ros(node: Node) -> None:
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


def main(args=None) -> None:
    rclpy.init(args=args)

    shared_state: dict = {}
    lock = threading.Lock()

    node = BridgeNode(shared_state, lock)

    # Read server params off the same node.
    node.declare_parameter("ws_host", "0.0.0.0")
    node.declare_parameter("ws_port", 9090)
    node.declare_parameter("ws_rate_hz", 30.0)
    host = node.get_parameter("ws_host").value
    port = int(node.get_parameter("ws_port").value)
    rate = float(node.get_parameter("ws_rate_hz").value)

    ros_thread = threading.Thread(target=_spin_ros, args=(node,), daemon=True)
    ros_thread.start()

    server = WsServer(host, port, rate, shared_state, lock)
    try:
        asyncio.run(server.run())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
