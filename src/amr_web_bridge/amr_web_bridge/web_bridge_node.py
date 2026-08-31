#!/usr/bin/env python3

from __future__ import annotations

import asyncio
from datetime import datetime, timezone
import json
import threading
from typing import Any

import rclpy
from rclpy.node import Node
import websockets


class WebBridgeNode(Node):

    def __init__(self) -> None:
        super().__init__('amr_web_bridge')

        self.declare_parameter(
            'server_url',
            'ws://localhost:8000',
        )
        self.declare_parameter(
            'robot_id',
            'robot01',
        )
        self.declare_parameter(
            'heartbeat_period',
            5.0,
        )
        self.declare_parameter(
            'reconnect_delay',
            3.0,
        )

        self.server_url = str(
            self.get_parameter(
                'server_url'
            ).value
        ).rstrip('/')

        self.robot_id = str(
            self.get_parameter(
                'robot_id'
            ).value
        )

        self.heartbeat_period = float(
            self.get_parameter(
                'heartbeat_period'
            ).value
        )

        self.reconnect_delay = float(
            self.get_parameter(
                'reconnect_delay'
            ).value
        )

        self.websocket_uri = (
            f'{self.server_url}/ws/robots/'
            f'{self.robot_id}'
        )

        self.stop_requested = threading.Event()
        self.asyncio_loop: asyncio.AbstractEventLoop | None = (
            None
        )
        self.websocket: Any = None

        self.worker_thread = threading.Thread(
            target=self.run_asyncio_thread,
            name='amr-websocket-thread',
            daemon=True,
        )
        self.worker_thread.start()

        self.get_logger().info(
            'AMR Web Bridge started'
        )
        self.get_logger().info(
            f'Robot ID: {self.robot_id}'
        )
        self.get_logger().info(
            f'FastAPI WebSocket: {self.websocket_uri}'
        )

    @staticmethod
    def utc_timestamp() -> str:
        return datetime.now(
            timezone.utc
        ).isoformat()

    def run_asyncio_thread(self) -> None:
        try:
            asyncio.run(
                self.connection_supervisor()
            )
        except Exception as error:
            if not self.stop_requested.is_set():
                self.get_logger().error(
                    f'WebSocket thread failed: {error}'
                )
        finally:
            self.asyncio_loop = None

    async def connection_supervisor(self) -> None:
        self.asyncio_loop = (
            asyncio.get_running_loop()
        )

        while not self.stop_requested.is_set():
            try:
                self.get_logger().info(
                    'Connecting to FastAPI...'
                )

                async with websockets.connect(
                    self.websocket_uri,
                    open_timeout=5,
                    ping_interval=20,
                    ping_timeout=20,
                ) as websocket:
                    self.websocket = websocket

                    self.get_logger().info(
                        'Connected to FastAPI WebSocket'
                    )

                    heartbeat_task = (
                        asyncio.create_task(
                            self.heartbeat_loop(
                                websocket
                            )
                        )
                    )

                    try:
                        await self.receive_loop(
                            websocket
                        )
                    finally:
                        heartbeat_task.cancel()

                        try:
                            await heartbeat_task
                        except asyncio.CancelledError:
                            pass

            except Exception as error:
                if not self.stop_requested.is_set():
                    self.get_logger().warning(
                        'WebSocket disconnected: '
                        f'{error}'
                    )

            finally:
                self.websocket = None

            if not self.stop_requested.is_set():
                self.get_logger().info(
                    'Reconnecting in '
                    f'{self.reconnect_delay:.1f} s'
                )
                await asyncio.sleep(
                    self.reconnect_delay
                )

    async def heartbeat_loop(
        self,
        websocket: Any,
    ) -> None:
        while not self.stop_requested.is_set():
            await asyncio.sleep(
                self.heartbeat_period
            )

            message = {
                'type': 'heartbeat',
                'timestamp': self.utc_timestamp(),
            }

            await websocket.send(
                json.dumps(message)
            )

    async def receive_loop(
        self,
        websocket: Any,
    ) -> None:
        async for raw_message in websocket:
            try:
                message = json.loads(
                    raw_message
                )
            except json.JSONDecodeError:
                self.get_logger().warning(
                    'Received invalid JSON'
                )
                continue

            await self.handle_message(
                websocket,
                message,
            )

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
            self.get_logger().debug(
                'Heartbeat acknowledged'
            )

        elif message_type == 'command':
            command_id = message.get(
                'command_id'
            )
            command_name = message.get(
                'command'
            )

            self.get_logger().warning(
                'Command received but Nav2 integration '
                f'is not enabled yet: {command_name}'
            )

            acknowledgement = {
                'type': 'command_ack',
                'command_id': command_id,
                'accepted': False,
                'detail': (
                    'ROS 2 bridge connected, but Nav2 '
                    'integration is not enabled yet'
                ),
            }

            await websocket.send(
                json.dumps(acknowledgement)
            )

        elif message_type == 'command_ack_received':
            self.get_logger().info(
                'FastAPI received command '
                'acknowledgement'
            )

        elif message_type == 'error':
            self.get_logger().warning(
                'FastAPI error: '
                f"{message.get('code')} - "
                f"{message.get('detail')}"
            )

        else:
            self.get_logger().debug(
                'Unhandled WebSocket message: '
                f'{message_type}'
            )

    def destroy_node(self) -> bool:
        self.stop_requested.set()

        if (
            self.asyncio_loop is not None
            and self.websocket is not None
        ):
            try:
                close_future = (
                    asyncio.run_coroutine_threadsafe(
                        self.websocket.close(),
                        self.asyncio_loop,
                    )
                )
                close_future.result(timeout=2.0)
            except Exception:
                pass

        if self.worker_thread.is_alive():
            self.worker_thread.join(
                timeout=3.0
            )

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
