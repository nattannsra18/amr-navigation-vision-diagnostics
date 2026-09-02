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
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
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
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('diagnostics_topic', '/diagnostics')
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
        self.odom_topic = str(
            self.get_parameter('odom_topic').value
        )
        self.diagnostics_topic = str(
            self.get_parameter('diagnostics_topic').value
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
        self.velocity_lock = threading.Lock()
        self.diagnostics_lock = threading.Lock()
        self.command_lock = threading.Lock()
        self.latest_telemetry: dict[str, Any] | None = None
        self.latest_map: dict[str, Any] | None = None
        self.latest_diagnostics: dict[str, Any] | None = None
        self.latest_velocity: (
            dict[str, float] | None
        ) = None
        self.map_revision = 0
        self.diagnostics_revision = 0
        self.command_queue: Queue[
            dict[str, Any]
        ] = Queue()
        self.cancel_queue: Queue[
            dict[str, Any]
        ] = Queue()
        self.pending_command_ids: set[str] = set()
        self.pending_cancel_requests: dict[
            str,
            dict[str, Any],
        ] = {}
        self.cancelled_task_ids: set[str] = set()
        self.active_command: (
            dict[str, Any] | None
        ) = None
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
        self.odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=(
                ReliabilityPolicy.BEST_EFFORT
            ),
            durability=DurabilityPolicy.VOLATILE,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            self.odom_qos,
        )
        self.diagnostics_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.diagnostics_subscription = self.create_subscription(
            DiagnosticArray,
            self.diagnostics_topic,
            self.diagnostics_callback,
            self.diagnostics_qos,
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
        self.get_logger().info(
            f'Odometry topic: {self.odom_topic}'
        )
        self.get_logger().info(
            f'Diagnostics topic: {self.diagnostics_topic}'
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

    def odom_callback(
        self,
        message: Odometry,
    ) -> None:
        twist = message.twist.twist

        linear_velocity = math.hypot(
            float(twist.linear.x),
            float(twist.linear.y),
        )
        angular_velocity = float(
            twist.angular.z
        )

        if not all(
            math.isfinite(value)
            for value in (
                linear_velocity,
                angular_velocity,
            )
        ):
            return

        with self.velocity_lock:
            self.latest_velocity = {
                'linear_velocity': linear_velocity,
                'angular_velocity': angular_velocity,
            }

    @staticmethod
    def normalize_diagnostic_level(level: Any) -> int | None:
        if isinstance(level, (bytes, bytearray, memoryview)):
            raw_bytes = bytes(level)
            return raw_bytes[0] if raw_bytes else None

        try:
            return int(level)
        except (TypeError, ValueError, OverflowError):
            return None

    def diagnostic_level_name(self, level: Any) -> str:
        normalized_level = WebBridgeNode.normalize_diagnostic_level(
            level
        )
        level_name = {
            0: 'OK',
            1: 'WARN',
            2: 'ERROR',
            3: 'STALE',
        }.get(normalized_level)

        if level_name is None:
            self.get_logger().warning(
                'Unknown or empty diagnostic level '
                f'{level!r}; using STALE'
            )
            return 'STALE'

        return level_name

    @staticmethod
    def diagnostic_timestamp(
        message: DiagnosticArray,
    ) -> str | None:
        stamp = message.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return None

        try:
            timestamp = (
                float(stamp.sec)
                + float(stamp.nanosec) / 1e9
            )
            return datetime.fromtimestamp(
                timestamp,
                tz=timezone.utc,
            ).isoformat()
        except (OverflowError, OSError, ValueError):
            return None

    def diagnostics_callback(
        self,
        message: DiagnosticArray,
    ) -> None:
        diagnostics = {
            'type': 'diagnostics',
            'timestamp': self.diagnostic_timestamp(message),
            'statuses': [
                {
                    'name': status.name,
                    'level': self.diagnostic_level_name(
                        status.level
                    ),
                    'message': status.message,
                    'hardware_id': status.hardware_id,
                    'values': [
                        {
                            'key': value.key,
                            'value': value.value,
                        }
                        for value in status.values
                    ],
                }
                for status in message.status
            ],
        }

        with self.diagnostics_lock:
            self.latest_diagnostics = diagnostics
            self.diagnostics_revision += 1

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
                        asyncio.create_task(
                            self.diagnostics_loop(websocket)
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

    async def diagnostics_loop(
        self,
        websocket: Any,
    ) -> None:
        sent_revision = -1

        while not self.stop_requested.is_set():
            await asyncio.sleep(1.0)

            with self.diagnostics_lock:
                revision = self.diagnostics_revision
                diagnostics = (
                    {
                        **self.latest_diagnostics,
                        'statuses': [
                            {
                                **status,
                                'values': [
                                    dict(value)
                                    for value
                                    in status['values']
                                ],
                            }
                            for status
                            in self.latest_diagnostics[
                                'statuses'
                            ]
                        ],
                    }
                    if self.latest_diagnostics is not None
                    else None
                )

            if (
                diagnostics is None
                or revision == sent_revision
            ):
                continue

            await self.send_json(
                websocket,
                diagnostics,
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
        elif message_type == 'cancel_navigation':
            await self.queue_navigation_cancel(
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
        elif (
            message_type
            == 'navigation_cancelled_received'
        ):
            self.get_logger().info(
                'FastAPI confirmed task cancellation: '
                f"{message.get('task_id')}"
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

    async def queue_navigation_cancel(
        self,
        websocket: Any,
        message: dict[str, Any],
    ) -> None:
        cancel_id = message.get('cancel_id')
        task_id = message.get('task_id')

        valid_request = (
            isinstance(cancel_id, str)
            and bool(cancel_id)
            and isinstance(task_id, str)
            and bool(task_id)
            and cancel_id.startswith(
                f'{task_id}:cancel:'
            )
        )

        if not valid_request:
            await self.send_json(
                websocket,
                {
                    'type': 'navigation_cancelled',
                    'cancel_id': cancel_id,
                    'task_id': task_id,
                    'cancelled': False,
                    'detail': (
                        'Invalid navigation '
                        'cancellation request'
                    ),
                },
            )
            return

        self.cancel_queue.put(message)

        self.get_logger().info(
            'Queued Nav2 cancellation '
            f'{cancel_id}'
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

    def process_cancel_queue(self) -> bool:
        try:
            cancel_request = (
                self.cancel_queue.get_nowait()
            )
        except Empty:
            return False

        cancel_id = str(
            cancel_request['cancel_id']
        )
        task_id = str(
            cancel_request['task_id']
        )

        with self.command_lock:
            self.pending_cancel_requests[
                task_id
            ] = cancel_request
            active_command = self.active_command
            active_goal_handle = (
                self.active_goal_handle
            )

        if active_command is None:
            with self.command_lock:
                self.cancelled_task_ids.add(
                    task_id
                )
                self.pending_cancel_requests.pop(
                    task_id,
                    None,
                )

            self.get_logger().info(
                'No active Nav2 goal for '
                f'{task_id}; cancellation confirmed'
            )
            self.send_navigation_cancelled(
                cancel_request,
                True,
                'No active Nav2 goal remains',
            )
            return True

        active_task_id = str(
            active_command.get('task_id')
        )

        if active_task_id != task_id:
            with self.command_lock:
                self.pending_cancel_requests.pop(
                    task_id,
                    None,
                )

            self.get_logger().warning(
                f'Cannot cancel {task_id}; '
                f'active task is {active_task_id}'
            )
            self.send_navigation_cancelled(
                cancel_request,
                False,
                (
                    'Cancellation task does not match '
                    'the active Nav2 goal'
                ),
            )
            return True

        if active_goal_handle is None:
            self.get_logger().info(
                'Waiting for Nav2 goal handle before '
                f'cancelling {cancel_id}'
            )
            return True

        self.request_goal_cancellation(
            active_command,
            active_goal_handle,
            cancel_request,
        )
        return True

    def request_goal_cancellation(
        self,
        command: dict[str, Any],
        goal_handle: Any,
        cancel_request: dict[str, Any],
    ) -> None:
        command_id = str(
            command['command_id']
        )
        task_id = str(
            command['task_id']
        )
        cancel_id = str(
            cancel_request['cancel_id']
        )

        self.get_logger().info(
            'Requesting Nav2 cancellation '
            f'{cancel_id} for {command_id}'
        )

        try:
            cancel_future = (
                goal_handle.cancel_goal_async()
            )
        except Exception as error:
            detail = (
                'Failed to request Nav2 '
                f'cancellation: {error}'
            )
            self.get_logger().error(detail)

            with self.command_lock:
                self.pending_cancel_requests.pop(
                    task_id,
                    None,
                )

            self.send_navigation_cancelled(
                cancel_request,
                False,
                detail,
            )
            return

        cancel_future.add_done_callback(
            lambda future: (
                self.cancel_response_callback(
                    future,
                    command,
                    cancel_request,
                )
            )
        )

    def cancel_response_callback(
        self,
        future: Any,
        command: dict[str, Any],
        cancel_request: dict[str, Any],
    ) -> None:
        task_id = str(
            command['task_id']
        )
        cancel_id = str(
            cancel_request['cancel_id']
        )

        try:
            response = future.result()
        except Exception as error:
            detail = (
                'Failed to receive Nav2 '
                f'cancellation response: {error}'
            )
            self.get_logger().error(detail)

            with self.command_lock:
                self.pending_cancel_requests.pop(
                    task_id,
                    None,
                )

            self.send_navigation_cancelled(
                cancel_request,
                False,
                detail,
            )
            return

        if not response.goals_canceling:
            detail = (
                'Nav2 did not accept the '
                'cancellation request'
            )
            self.get_logger().warning(
                f'{detail}: {cancel_id}'
            )

            with self.command_lock:
                self.pending_cancel_requests.pop(
                    task_id,
                    None,
                )

            self.send_navigation_cancelled(
                cancel_request,
                False,
                detail,
            )
            return

        self.get_logger().info(
            'Nav2 accepted cancellation request '
            f'{cancel_id}; waiting for final result'
        )

    def process_command_queue(self) -> None:
        # Cancellation always takes priority over
        # starting another navigation command.
        if self.process_cancel_queue():
            return

        with self.command_lock:
            if self.active_command is not None:
                return

        try:
            command = self.command_queue.get_nowait()
        except Empty:
            return

        command_id = str(
            command['command_id']
        )
        task_id = str(
            command['task_id']
        )

        with self.command_lock:
            skip_cancelled_command = (
                task_id in self.cancelled_task_ids
            )
            self.pending_command_ids.discard(
                command_id
            )

            if skip_cancelled_command:
                self.cancelled_task_ids.discard(
                    task_id
                )
            else:
                self.active_command = command

        if skip_cancelled_command:
            self.get_logger().info(
                'Skipping queued Nav2 command '
                f'{command_id} because task '
                f'{task_id} was cancelled'
            )
            return

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
            feedback_callback=(
                lambda feedback_message: (
                    self.navigation_feedback_callback(
                        feedback_message,
                        command,
                    )
                )
            ),
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
        command_id = str(
            command['command_id']
        )
        task_id = str(
            command['task_id']
        )

        try:
            goal_handle = future.result()
        except Exception as error:
            detail = (
                f'Failed to send Nav2 goal: {error}'
            )
            self.get_logger().error(detail)
            self.send_command_ack(
                command,
                False,
                detail,
            )

            with self.command_lock:
                cancel_request = (
                    self.pending_cancel_requests.pop(
                        task_id,
                        None,
                    )
                )

            if cancel_request is not None:
                self.send_navigation_cancelled(
                    cancel_request,
                    True,
                    (
                        'Nav2 goal never became active: '
                        f'{detail}'
                    ),
                )
            else:
                self.send_navigation_result(
                    command,
                    'aborted',
                    detail,
                )

            self.clear_active_command(
                command_id
            )
            return

        if not goal_handle.accepted:
            detail = (
                'Nav2 rejected the navigation goal'
            )
            self.get_logger().warning(detail)
            self.send_command_ack(
                command,
                False,
                detail,
            )

            with self.command_lock:
                cancel_request = (
                    self.pending_cancel_requests.pop(
                        task_id,
                        None,
                    )
                )

            if cancel_request is not None:
                self.send_navigation_cancelled(
                    cancel_request,
                    True,
                    (
                        'Nav2 goal was not active '
                        'because it was rejected'
                    ),
                )
            else:
                self.send_navigation_result(
                    command,
                    'aborted',
                    detail,
                )

            self.clear_active_command(
                command_id
            )
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

        result_future = (
            goal_handle.get_result_async()
        )
        result_future.add_done_callback(
            lambda result: (
                self.navigation_result_callback(
                    result,
                    command,
                )
            )
        )

        with self.command_lock:
            cancel_request = (
                self.pending_cancel_requests.get(
                    task_id
                )
            )

        if cancel_request is not None:
            self.request_goal_cancellation(
                command,
                goal_handle,
                cancel_request,
            )

    def navigation_feedback_callback(
        self,
        feedback_message: Any,
        command: dict[str, Any],
    ) -> None:
        now_ns = self.get_clock().now().nanoseconds

        if (
            now_ns - self.last_feedback_log_ns
            < 2_000_000_000
        ):
            return

        self.last_feedback_log_ns = now_ns

        feedback = feedback_message.feedback
        current_pose = feedback.current_pose
        pose = current_pose.pose
        orientation = pose.orientation

        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )
        yaw = math.atan2(
            sin_yaw,
            cos_yaw,
        )

        navigation_duration = (
            feedback.navigation_time
        )
        navigation_time_seconds = (
            float(navigation_duration.sec)
            + (
                float(navigation_duration.nanosec)
                / 1_000_000_000.0
            )
        )

        estimated_duration = (
            feedback.estimated_time_remaining
        )
        estimated_time_remaining_seconds = (
            float(estimated_duration.sec)
            + (
                float(estimated_duration.nanosec)
                / 1_000_000_000.0
            )
        )

        distance_remaining = max(
            0.0,
            float(feedback.distance_remaining),
        )
        navigation_time_seconds = max(
            0.0,
            navigation_time_seconds,
        )
        estimated_time_remaining_seconds = max(
            0.0,
            estimated_time_remaining_seconds,
        )

        x = float(pose.position.x)
        y = float(pose.position.y)

        numeric_values = (
            distance_remaining,
            navigation_time_seconds,
            estimated_time_remaining_seconds,
            x,
            y,
            yaw,
        )

        if not all(
            math.isfinite(value)
            for value in numeric_values
        ):
            self.get_logger().warning(
                'Ignoring non-finite Nav2 feedback'
            )
            return

        with self.velocity_lock:
            velocity = (
                dict(self.latest_velocity)
                if self.latest_velocity is not None
                else None
            )

        self.get_logger().debug(
            'Nav2 feedback: '
            f'{distance_remaining:.2f} m remaining, '
            'ETA '
            f'{estimated_time_remaining_seconds:.1f} s'
        )

        self.send_from_ros(
            {
                'type': 'navigation_feedback',
                'command_id': command.get(
                    'command_id'
                ),
                'task_id': command.get(
                    'task_id'
                ),
                'stage': command.get(
                    'stage'
                ),
                'distance_remaining': (
                    distance_remaining
                ),
                'navigation_time_seconds': (
                    navigation_time_seconds
                ),
                (
                    'estimated_time_'
                    'remaining_seconds'
                ): (
                    estimated_time_remaining_seconds
                ),
                'number_of_recoveries': max(
                    0,
                    int(feedback.number_of_recoveries),
                ),
                'linear_velocity': (
                    velocity['linear_velocity']
                    if velocity is not None
                    else None
                ),
                'angular_velocity': (
                    velocity['angular_velocity']
                    if velocity is not None
                    else None
                ),
                'current_pose': {
                    'frame_id': (
                        current_pose.header.frame_id
                        or 'map'
                    ),
                    'x': x,
                    'y': y,
                    'yaw': yaw,
                },
                'timestamp': self.utc_timestamp(),
            }
        )

    def navigation_result_callback(
        self,
        future: Any,
        command: dict[str, Any],
    ) -> None:
        command_id = str(
            command['command_id']
        )
        task_id = str(
            command['task_id']
        )

        try:
            wrapped_result = future.result()
            status = wrapped_result.status
            result = wrapped_result.result
        except Exception as error:
            detail = (
                'Failed to receive Nav2 result: '
                f'{error}'
            )
            self.get_logger().error(detail)

            with self.command_lock:
                cancel_request = (
                    self.pending_cancel_requests.pop(
                        task_id,
                        None,
                    )
                )

            if cancel_request is not None:
                self.send_navigation_cancelled(
                    cancel_request,
                    False,
                    detail,
                )
            else:
                self.send_navigation_result(
                    command,
                    'aborted',
                    detail,
                )

            self.clear_active_command(
                command_id
            )
            return

        with self.command_lock:
            cancel_request = (
                self.pending_cancel_requests.pop(
                    task_id,
                    None,
                )
            )

        if cancel_request is not None:
            if (
                status
                == GoalStatus.STATUS_CANCELED
            ):
                detail = (
                    'Nav2 goal cancellation completed'
                )
            elif (
                status
                == GoalStatus.STATUS_SUCCEEDED
            ):
                detail = (
                    'Nav2 goal finished before the '
                    'cancellation took effect'
                )
            else:
                error_message = getattr(
                    result,
                    'error_msg',
                    '',
                )
                detail = (
                    error_message
                    or (
                        'Nav2 goal stopped before '
                        'cancellation completed'
                    )
                )

            self.get_logger().info(
                'Navigation stopped for cancelled '
                f'task {task_id}: {detail}'
            )
            self.send_navigation_cancelled(
                cancel_request,
                True,
                detail,
            )
            self.clear_active_command(
                command_id
            )
            return

        if (
            status
            == GoalStatus.STATUS_SUCCEEDED
        ):
            navigation_status = 'succeeded'
            detail = 'Nav2 goal succeeded'
            self.get_logger().info(
                'Navigation succeeded for '
                f'{command_id}'
            )

        elif (
            status
            == GoalStatus.STATUS_CANCELED
        ):
            navigation_status = 'canceled'
            detail = 'Nav2 goal was canceled'
            self.get_logger().warning(
                'Navigation canceled for '
                f'{command_id}'
            )

        else:
            navigation_status = 'aborted'
            error_message = getattr(
                result,
                'error_msg',
                '',
            )
            detail = (
                error_message
                or 'Nav2 goal was aborted'
            )
            self.get_logger().error(
                'Navigation aborted for '
                f'{command_id}: {detail}'
            )

        self.send_navigation_result(
            command,
            navigation_status,
            detail,
        )
        self.clear_active_command(
            command_id
        )

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

    def send_navigation_cancelled(
        self,
        cancel_request: dict[str, Any],
        cancelled: bool,
        detail: str,
    ) -> None:
        self.send_from_ros(
            {
                'type': 'navigation_cancelled',
                'cancel_id': (
                    cancel_request.get(
                        'cancel_id'
                    )
                ),
                'task_id': (
                    cancel_request.get(
                        'task_id'
                    )
                ),
                'cancelled': cancelled,
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
