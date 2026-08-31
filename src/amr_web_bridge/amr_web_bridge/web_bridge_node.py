#!/usr/bin/env python3

from __future__ import annotations

import asyncio
from datetime import datetime, timezone
import json
import math
from queue import Empty, Queue
import threading
from typing import Any

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
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
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('battery_percent', 100)
        self.declare_parameter(
            'navigate_action',
            '/navigate_to_pose',
        )

        self.server_url = str(
            self.get_parameter('server_url').value
        ).rstrip('/')
        self.robot_id = str(
            self.get_parameter('robot_id').value
        )
        self.heartbeat_period = float(
            self.get_parameter('heartbeat_period').value
        )
        self.telemetry_period = float(
            self.get_parameter('telemetry_period').value
        )
        self.reconnect_delay = float(
            self.get_parameter('reconnect_delay').value
        )
        self.pose_topic = str(
            self.get_parameter('pose_topic').value
        )
        self.map_topic = str(
            self.get_parameter('map_topic').value
        )
        self.battery_percent = int(
            self.get_parameter('battery_percent').value
        )
        self.navigate_action = str(
            self.get_parameter('navigate_action').value
        )
        self.websocket_uri = (
            f'{self.server_url}/ws/robots/{self.robot_id}'
        )

        self.stop_requested = threading.Event()
        self.telemetry_lock = threading.Lock()
        self.map_lock = threading.Lock()
        self.command_lock = threading.Lock()
        self.latest_telemetry: dict[str, Any] | None = None
        self.latest_map: dict[str, Any] | None = None
        self.map_revision = 0
        self.command_queue: Queue[dict[str, Any]] = Queue()
        self.pending_command_ids: set[str] = set()
        self.active_command: dict[str, Any] | None = None
        self.active_goal_handle: Any = None
        self.last_feedback_log_ns = 0

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
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            self.pose_qos,
        )

        self.navigation_client = ActionClient(
            self,
            NavigateToPose,
            self.navigate_action,
        )
        self.command_timer = self.create_timer(
            0.1,
            self.process_command_queue,
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
        self.get_logger().info(f'Map topic: {self.map_topic}')
        self.get_logger().info(
            f'Nav2 action: {self.navigate_action}'
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

    def map_callback(self, message: OccupancyGrid) -> None:
        origin = message.info.origin
        orientation = origin.orientation

        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )
        origin_yaw = math.atan2(sin_yaw, cos_yaw)
        stamp = message.header.stamp

        map_payload = {
            'frame_id': message.header.frame_id or 'map',
            'resolution': float(message.info.resolution),
            'width': int(message.info.width),
            'height': int(message.info.height),
            'origin_x': float(origin.position.x),
            'origin_y': float(origin.position.y),
            'origin_yaw': float(origin_yaw),
            'data': [int(value) for value in message.data],
            'timestamp': (
                f'{stamp.sec}.{stamp.nanosec:09d}'
            ),
        }

        with self.map_lock:
            self.latest_map = map_payload
            self.map_revision += 1

        self.get_logger().info(
            'Received ROS map '
            f'{message.info.width}x{message.info.height}'
        )

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
                        asyncio.create_task(
                            self.map_loop(websocket)
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

    async def map_loop(self, websocket: Any) -> None:
        sent_revision = -1

        while not self.stop_requested.is_set():
            await asyncio.sleep(0.5)

            with self.map_lock:
                revision = self.map_revision
                map_payload = (
                    dict(self.latest_map)
                    if self.latest_map is not None
                    else None
                )

            if (
                map_payload is None
                or revision == sent_revision
            ):
                continue

            await self.send_json(
                websocket,
                {
                    'type': 'map',
                    'data': map_payload,
                },
            )
            sent_revision = revision

    async def send_json(
        self,
        websocket: Any,
        message: dict[str, Any],
    ) -> None:
        if self.send_lock is None:
            return

        async with self.send_lock:
            await websocket.send(json.dumps(message))

    async def send_current_json(
        self,
        message: dict[str, Any],
    ) -> None:
        websocket = self.websocket
        if websocket is None:
            return

        await self.send_json(websocket, message)

    def send_from_ros(
        self,
        message: dict[str, Any],
    ) -> None:
        loop = self.asyncio_loop
        if loop is None or self.websocket is None:
            self.get_logger().warning(
                'Cannot send message: WebSocket is disconnected'
            )
            return

        future = asyncio.run_coroutine_threadsafe(
            self.send_current_json(message),
            loop,
        )
        future.add_done_callback(self.send_future_callback)

    def send_future_callback(self, future: Any) -> None:
        try:
            future.result()
        except Exception as error:
            if not self.stop_requested.is_set():
                self.get_logger().warning(
                    f'Failed to send WebSocket message: {error}'
                )

    async def receive_loop(self, websocket: Any) -> None:
        async for raw_message in websocket:
            try:
                message = json.loads(raw_message)
            except json.JSONDecodeError:
                self.get_logger().warning('Received invalid JSON')
                continue

            if not isinstance(message, dict):
                self.get_logger().warning(
                    'Received WebSocket message is not an object'
                )
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
        elif message_type == 'map_ack':
            self.get_logger().info(
                'Map acknowledged by FastAPI: '
                f"revision {message.get('revision')}"
            )
        elif message_type == 'command':
            await self.queue_navigation_command(
                websocket,
                message,
            )
        elif message_type == 'command_ack_received':
            self.get_logger().info(
                'FastAPI received command acknowledgement'
            )
        elif message_type == 'navigation_result_received':
            self.get_logger().info(
                'FastAPI applied navigation result: '
                f"{message.get('task_status')}"
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

    async def queue_navigation_command(
        self,
        websocket: Any,
        message: dict[str, Any],
    ) -> None:
        command_id = message.get('command_id')
        command_name = message.get('command')
        task_id = message.get('task_id')
        stage = message.get('stage')
        target = message.get('target')

        valid_target = (
            isinstance(target, dict)
            and isinstance(target.get('frame_id'), str)
            and bool(target.get('frame_id'))
            and isinstance(target.get('x'), (int, float))
            and isinstance(target.get('y'), (int, float))
            and isinstance(target.get('yaw'), (int, float))
        )
        valid_command = (
            isinstance(command_id, str)
            and bool(command_id)
            and command_name == 'navigate_to_pose'
            and isinstance(task_id, str)
            and bool(task_id)
            and stage in {'pickup', 'destination'}
            and valid_target
        )

        if not valid_command:
            await self.send_json(
                websocket,
                {
                    'type': 'command_ack',
                    'command_id': command_id,
                    'accepted': False,
                    'detail': 'Invalid navigate_to_pose command',
                },
            )
            return

        with self.command_lock:
            active_id = (
                self.active_command.get('command_id')
                if self.active_command is not None
                else None
            )
            duplicate = (
                command_id == active_id
                or command_id in self.pending_command_ids
            )

            if not duplicate:
                self.pending_command_ids.add(command_id)

        if duplicate:
            await self.send_json(
                websocket,
                {
                    'type': 'command_ack',
                    'command_id': command_id,
                    'accepted': True,
                    'detail': 'Command is already queued or active',
                },
            )
            return

        self.command_queue.put(message)
        self.get_logger().info(
            f'Queued Nav2 command {command_id}'
        )

    def process_command_queue(self) -> None:
        with self.command_lock:
            if self.active_command is not None:
                return

        try:
            command = self.command_queue.get_nowait()
        except Empty:
            return

        command_id = str(command['command_id'])
        with self.command_lock:
            self.pending_command_ids.discard(command_id)
            self.active_command = command

        if not self.navigation_client.wait_for_server(
            timeout_sec=1.0
        ):
            detail = 'Nav2 NavigateToPose server is unavailable'
            self.get_logger().error(detail)
            self.send_command_ack(command, False, detail)
            self.send_navigation_result(
                command,
                'aborted',
                detail,
            )
            self.clear_active_command(command_id)
            return

        goal = self.build_navigation_goal(command)
        self.get_logger().info(
            'Sending Nav2 goal '
            f"{command_id} to {command['target']}"
        )

        goal_future = self.navigation_client.send_goal_async(
            goal,
            feedback_callback=self.navigation_feedback_callback,
        )
        goal_future.add_done_callback(
            lambda future: self.goal_response_callback(
                future,
                command,
            )
        )

    def build_navigation_goal(
        self,
        command: dict[str, Any],
    ) -> NavigateToPose.Goal:
        target = command['target']
        yaw = float(target['yaw'])

        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = str(target['frame_id'])
        goal.pose.pose.position.x = float(target['x'])
        goal.pose.pose.position.y = float(target['y'])
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def goal_response_callback(
        self,
        future: Any,
        command: dict[str, Any],
    ) -> None:
        command_id = str(command['command_id'])

        try:
            goal_handle = future.result()
        except Exception as error:
            detail = f'Failed to send Nav2 goal: {error}'
            self.get_logger().error(detail)
            self.send_command_ack(command, False, detail)
            self.send_navigation_result(
                command,
                'aborted',
                detail,
            )
            self.clear_active_command(command_id)
            return

        if not goal_handle.accepted:
            detail = 'Nav2 rejected the navigation goal'
            self.get_logger().warning(detail)
            self.send_command_ack(command, False, detail)
            self.send_navigation_result(
                command,
                'aborted',
                detail,
            )
            self.clear_active_command(command_id)
            return

        with self.command_lock:
            self.active_goal_handle = goal_handle

        self.get_logger().info(
            f'Nav2 accepted command {command_id}'
        )
        self.send_command_ack(
            command,
            True,
            'Nav2 accepted the navigation goal',
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self.navigation_result_callback(
                result,
                command,
            )
        )

    def navigation_feedback_callback(self, feedback: Any) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_feedback_log_ns < 2_000_000_000:
            return

        self.last_feedback_log_ns = now_ns
        distance = feedback.feedback.distance_remaining
        self.get_logger().debug(
            f'Nav2 distance remaining: {distance:.2f} m'
        )

    def navigation_result_callback(
        self,
        future: Any,
        command: dict[str, Any],
    ) -> None:
        command_id = str(command['command_id'])

        try:
            wrapped_result = future.result()
            status = wrapped_result.status
            result = wrapped_result.result
        except Exception as error:
            detail = f'Failed to receive Nav2 result: {error}'
            self.get_logger().error(detail)
            self.send_navigation_result(
                command,
                'aborted',
                detail,
            )
            self.clear_active_command(command_id)
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            navigation_status = 'succeeded'
            detail = 'Nav2 goal succeeded'
            self.get_logger().info(
                f'Navigation succeeded for {command_id}'
            )
        elif status == GoalStatus.STATUS_CANCELED:
            navigation_status = 'canceled'
            detail = 'Nav2 goal was canceled'
            self.get_logger().warning(
                f'Navigation canceled for {command_id}'
            )
        else:
            navigation_status = 'aborted'
            error_message = getattr(result, 'error_msg', '')
            detail = error_message or 'Nav2 goal was aborted'
            self.get_logger().error(
                'Navigation aborted for '
                f'{command_id}: {detail}'
            )

        self.send_navigation_result(
            command,
            navigation_status,
            detail,
        )
        self.clear_active_command(command_id)

    def send_command_ack(
        self,
        command: dict[str, Any],
        accepted: bool,
        detail: str,
    ) -> None:
        self.send_from_ros(
            {
                'type': 'command_ack',
                'command_id': command.get('command_id'),
                'accepted': accepted,
                'detail': detail,
            }
        )

    def send_navigation_result(
        self,
        command: dict[str, Any],
        status: str,
        detail: str,
    ) -> None:
        self.send_from_ros(
            {
                'type': 'navigation_result',
                'command_id': command.get('command_id'),
                'task_id': command.get('task_id'),
                'stage': command.get('stage'),
                'status': status,
                'detail': detail,
            }
        )

    def clear_active_command(self, command_id: str) -> None:
        with self.command_lock:
            if (
                self.active_command is not None
                and self.active_command.get('command_id')
                == command_id
            ):
                self.active_command = None
                self.active_goal_handle = None

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
