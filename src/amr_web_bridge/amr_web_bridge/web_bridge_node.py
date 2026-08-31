#!/usr/bin/env python3

from __future__ import annotations

import asyncio
from datetime import datetime, timezone
import json
import math
import threading
from typing import Any

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
import websockets


class WebBridgeNode(Node):

    def __init__(self) -> None:
        super().__init__('amr_web_bridge')

        self.declare_parameter('server_url', 'ws://localhost:8000')
        self.declare_parameter('robot_id', 'robot01')
        self.declare_parameter('heartbeat_period', 5.0)
        self.declare_parameter('telemetry_period', 1.0)
        self.declare_parameter('reconnect_delay', 3.0)
        self.declare_parameter('pose_topic', '/amcl_pose')
        self.declare_parameter('battery_percent', 100)

        self.server_url = str(
            self.get_parameter('server_url').value
        ).rstrip('/')
        self.robot_id = str(self.get_parameter('robot_id').value)
        self.heartbeat_period = float(
            self.get_parameter('heartbeat_period').value
        )
        self.telemetry_period = float(
            self.get_parameter('telemetry_period').value
        )
        self.reconnect_delay = float(
            self.get_parameter('reconnect_delay').value
        )
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.battery_percent = int(
            self.get_parameter('battery_percent').value
        )
        self.websocket_uri = (
            f'{self.server_url}/ws/robots/{self.robot_id}'
        )

        self.stop_requested = threading.Event()
        self.telemetry_lock = threading.Lock()
        self.latest_telemetry: dict[str, Any] | None = None
        self.asyncio_loop: asyncio.AbstractEventLoop | None = None
        self.send_lock: asyncio.Lock | None = None
        self.websocket: Any = None

        self.pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_topic,
            self.pose_callback,
            self.pose_qos,
        )

        self.worker_thread = threading.Thread(
            target=self.run_asyncio_thread,
            name='amr-websocket-thread',
            daemon=True,
        )
        self.worker_thread.start()

        self.get_logger().info('AMR Web Bridge started')
        self.get_logger().info(f'Robot ID: {self.robot_id}')
        self.get_logger().info(
            f'FastAPI WebSocket: {self.websocket_uri}'
        )
        self.get_logger().info(
            f'Pose telemetry topic: {self.pose_topic}'
        )

    @staticmethod
    def utc_timestamp() -> str:
        return datetime.now(timezone.utc).isoformat()

    def run_asyncio_thread(self) -> None:
        try:
            asyncio.run(self.connection_supervisor())
        except Exception as error:
            if not self.stop_requested.is_set():
                self.get_logger().error(
                    f'WebSocket thread failed: {error}'
                )
        finally:
            self.asyncio_loop = None

    def pose_callback(
        self,
        message: PoseWithCovarianceStamped,
    ) -> None:
        pose = message.pose.pose
        orientation = pose.orientation

        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )
        yaw = math.atan2(sin_yaw, cos_yaw)

        telemetry = {
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'yaw': float(yaw),
            'battery': self.battery_percent,
            'frame_id': message.header.frame_id or 'map',
            'timestamp': self.utc_timestamp(),
        }

        with self.telemetry_lock:
            self.latest_telemetry = telemetry

    async def connection_supervisor(self) -> None:
        self.asyncio_loop = asyncio.get_running_loop()

        while not self.stop_requested.is_set():
            try:
                self.get_logger().info('Connecting to FastAPI...')

                async with websockets.connect(
                    self.websocket_uri,
                    open_timeout=5,
                    ping_interval=20,
                    ping_timeout=20,
                ) as websocket:
                    self.websocket = websocket
                    self.send_lock = asyncio.Lock()
                    self.get_logger().info(
                        'Connected to FastAPI WebSocket'
                    )

                    background_tasks = [
                        asyncio.create_task(
                            self.heartbeat_loop(websocket)
                        ),
                        asyncio.create_task(
                            self.telemetry_loop(websocket)
                        ),
                    ]

                    try:
                        await self.receive_loop(websocket)
                    finally:
                        for task in background_tasks:
                            task.cancel()

                        await asyncio.gather(
                            *background_tasks,
                            return_exceptions=True,
                        )

            except Exception as error:
                if not self.stop_requested.is_set():
                    self.get_logger().warning(
                        f'WebSocket disconnected: {error}'
                    )
            finally:
                self.websocket = None
                self.send_lock = None

            if not self.stop_requested.is_set():
                self.get_logger().info(
                    f'Reconnecting in {self.reconnect_delay:.1f} s'
                )
                await asyncio.sleep(self.reconnect_delay)

    async def heartbeat_loop(self, websocket: Any) -> None:
        while not self.stop_requested.is_set():
            await asyncio.sleep(self.heartbeat_period)
            await self.send_json(
                websocket,
                {
                    'type': 'heartbeat',
                    'timestamp': self.utc_timestamp(),
                },
            )

    async def telemetry_loop(self, websocket: Any) -> None:
        while not self.stop_requested.is_set():
            await asyncio.sleep(self.telemetry_period)

            with self.telemetry_lock:
                telemetry = (
                    dict(self.latest_telemetry)
                    if self.latest_telemetry is not None
                    else None
                )

            if telemetry is None:
                continue

            await self.send_json(
                websocket,
                {
                    'type': 'telemetry',
                    'data': telemetry,
                },
            )

    async def send_json(
        self,
        websocket: Any,
        message: dict[str, Any],
    ) -> None:
        if self.send_lock is None:
            return

        async with self.send_lock:
            await websocket.send(json.dumps(message))

    async def receive_loop(self, websocket: Any) -> None:
        async for raw_message in websocket:
            try:
                message = json.loads(raw_message)
            except json.JSONDecodeError:
                self.get_logger().warning('Received invalid JSON')
                continue

            await self.handle_message(websocket, message)

    async def handle_message(
        self,
        websocket: Any,
        message: dict[str, Any],
    ) -> None:
        message_type = message.get('type')

        if message_type == 'connection_ack':
            self.get_logger().info(
                'FastAPI acknowledged connection '
                f"for {message.get('robot_id')}"
            )
        elif message_type == 'heartbeat_ack':
            self.get_logger().debug('Heartbeat acknowledged')
        elif message_type == 'telemetry_ack':
            self.get_logger().debug('Telemetry acknowledged')
        elif message_type == 'command':
            await self.reject_command_until_nav2_ready(
                websocket,
                message,
            )
        elif message_type == 'command_ack_received':
            self.get_logger().info(
                'FastAPI received command acknowledgement'
            )
        elif message_type == 'error':
            self.get_logger().warning(
                'FastAPI error: '
                f"{message.get('code')} - "
                f"{message.get('detail')}"
            )
        else:
            self.get_logger().debug(
                f'Unhandled WebSocket message: {message_type}'
            )

    async def reject_command_until_nav2_ready(
        self,
        websocket: Any,
        message: dict[str, Any],
    ) -> None:
        command_name = message.get('command')
        self.get_logger().warning(
            'Command received but Nav2 integration '
            f'is not enabled yet: {command_name}'
        )

        acknowledgement = {
            'type': 'command_ack',
            'command_id': message.get('command_id'),
            'accepted': False,
            'detail': (
                'ROS 2 bridge connected, but Nav2 integration '
                'is not enabled yet'
            ),
        }
        await self.send_json(websocket, acknowledgement)

    def destroy_node(self) -> bool:
        self.stop_requested.set()

        if self.asyncio_loop is not None and self.websocket is not None:
            try:
                close_future = asyncio.run_coroutine_threadsafe(
                    self.websocket.close(),
                    self.asyncio_loop,
                )
                close_future.result(timeout=2.0)
            except Exception:
                pass

        if self.worker_thread.is_alive():
            self.worker_thread.join(timeout=3.0)

        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
