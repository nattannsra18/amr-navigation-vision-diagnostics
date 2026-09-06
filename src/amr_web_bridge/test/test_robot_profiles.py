from pathlib import Path

import yaml


PROFILE_DIR = Path(__file__).parents[1] / 'config' / 'profiles'


def parameters(name):
    payload = yaml.safe_load((PROFILE_DIR / name).read_text(encoding='utf-8'))
    return payload['amr_web_bridge']['ros__parameters']


def test_sim_profile_declares_identity_capabilities_and_standard_interfaces():
    profile = parameters('turtlebot3_waffle_sim.yaml')
    assert profile['robot_serial_number'] == 'SIM-0001'
    assert profile['profile_version'] == 'turtlebot3-waffle-sim-v1'
    assert profile['navigate_action'] == '/navigate_to_pose'
    assert profile['map_topic'] == '/map'
    assert profile['odom_topic'] == '/odom'


def test_physical_profile_is_an_explicit_non_secret_template():
    profile = parameters('scuttle_real.example.yaml')
    assert profile['robot_serial_number'] == 'REPLACE-WITH-CHASSIS-SERIAL'
    assert profile['profile_version'] == 'scuttle-real-v1'
    assert 'credential' not in profile
    assert 'token' not in profile
