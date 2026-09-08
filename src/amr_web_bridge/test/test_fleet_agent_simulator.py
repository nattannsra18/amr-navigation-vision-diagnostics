import asyncio
import json
from pathlib import Path

from amr_web_bridge.fleet_agent_simulator import (
    FleetAgentProfile,
    FleetAgentSimulator,
)


PACKAGE_ROOT = Path(__file__).parents[1]
PROFILE_DIR = PACKAGE_ROOT / 'config' / 'fleet_lab'


class Socket:
    def __init__(self):
        self.messages = []

    async def send(self, payload):
        self.messages.append(json.loads(payload))


def test_fleet_lab_profiles_have_isolated_identity_namespace_and_behavior():
    profiles = [
        FleetAgentProfile.load(PROFILE_DIR / name)
        for name in ('sim01.yaml', 'sim02.yaml', 'robot-test01.yaml')
    ]

    assert len({profile.serial_number for profile in profiles}) == 3
    assert len({profile.profile_version for profile in profiles}) == 3
    assert len({profile.ros_namespace for profile in profiles}) == 3
    assert {profile.active_map_id for profile in profiles} == {
        'warehouse_map', 'lab_map'
    }
    assert profiles[0].capabilities != profiles[1].capabilities
    assert profiles[1].capabilities != profiles[2].capabilities


def test_navigation_command_is_acknowledged_and_completed(tmp_path):
    profile = FleetAgentProfile.load(PROFILE_DIR / 'sim01.yaml')
    profile = FleetAgentProfile(
        **{
            **profile.__dict__,
            'navigation_delay_seconds': 0.1,
        }
    )
    agent = FleetAgentSimulator(
        profile,
        'http://localhost:8000',
        tmp_path / 'sim01.json',
        'bootstrap-token',
    )
    agent.robot_id = 'robot-sim01'
    socket = Socket()

    asyncio.run(agent._handle_message(socket, {
        'type': 'command',
        'command': 'navigate_to_pose',
        'command_id': 'TASK-1:pickup:command',
        'task_id': 'TASK-1',
        'stage': 'pickup',
        'target': {'x': 2.5, 'y': -1.0, 'yaw': 1.57},
    }))

    assert [message['type'] for message in socket.messages] == [
        'command_status', 'navigation_result'
    ]
    assert socket.messages[0]['robot_id'] == 'robot-sim01'
    assert socket.messages[1]['status'] == 'succeeded'
    assert (agent.x, agent.y, agent.yaw) == (2.5, -1.0, 1.57)


def test_route_preview_returns_two_reachable_segments(tmp_path):
    profile = FleetAgentProfile.load(PROFILE_DIR / 'sim02.yaml')
    agent = FleetAgentSimulator(
        profile,
        'http://localhost:8000',
        tmp_path / 'sim02.json',
        'bootstrap-token',
    )
    socket = Socket()

    asyncio.run(agent._handle_message(socket, {
        'type': 'route_preview_request',
        'request_id': 'preview-1',
        'start': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        'pickup': {'x': 1.0, 'y': 2.0, 'yaw': 0.5},
        'destination': {'x': 3.0, 'y': 4.0, 'yaw': 1.0},
    }))

    result = socket.messages[0]
    assert result['type'] == 'route_preview_result'
    assert result['status'] == 'available'
    assert result['pickup_path'][1] == {'x': 1.0, 'y': 2.0, 'yaw': 0.5}
    assert result['delivery_path'][1] == {'x': 3.0, 'y': 4.0, 'yaw': 1.0}


def test_launcher_uses_one_credential_file_per_agent():
    launcher = (PACKAGE_ROOT.parents[1] / 'scripts' / 'fleet_lab.sh').read_text(
        encoding='utf-8'
    )

    assert 'agents=(sim01 sim02 robot-test01)' in launcher
    assert 'credentials/${agent}.json' in launcher
    assert 'reset-credentials' in launcher
