from pathlib import Path
import stat

from amr_web_bridge.agent_identity import (
    AgentCredential,
    AgentCredentialStore,
    EnrollmentClient,
    machine_fingerprint,
    verification_fingerprint,
)


def test_credential_store_round_trip_is_owner_readable(tmp_path):
    store = AgentCredentialStore(tmp_path / 'identity' / 'credential.json')
    credential = AgentCredential('robot-uuid', 'secret-value', 2)
    store.save(credential)
    assert store.load() == credential
    assert stat.S_IMODE(store.path.stat().st_mode) == 0o600
    assert 'secret-value' in store.path.read_text(encoding='utf-8')


def test_invalid_credential_file_is_ignored(tmp_path):
    path = tmp_path / 'credential.json'
    path.write_text('{not-json', encoding='utf-8')
    assert AgentCredentialStore(path).load() is None


def test_websocket_url_is_converted_to_http_control_plane():
    assert EnrollmentClient('ws://localhost:8000', 'bootstrap').base_url == 'http://localhost:8000'
    assert EnrollmentClient('wss://robot.example', 'bootstrap').base_url == 'https://robot.example'


def test_machine_fingerprint_is_stable_and_verification_digest_matches_server(tmp_path):
    machine_id = Path(tmp_path / 'machine-id')
    machine_id.write_text('machine-a\n', encoding='utf-8')
    fingerprint = machine_fingerprint('SIM-001', machine_id)
    assert fingerprint == machine_fingerprint('SIM-001', machine_id)
    assert fingerprint != machine_fingerprint('SIM-002', machine_id)
    assert len(verification_fingerprint(fingerprint)) == 64
