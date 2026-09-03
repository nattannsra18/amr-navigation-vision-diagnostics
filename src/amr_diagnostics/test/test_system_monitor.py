from amr_diagnostics.system_monitor import AmrSystemMonitor


class StubMonitor:
    stale_timeout_seconds = 2.0
    stationary_linear_velocity_threshold = 0.01
    stationary_angular_velocity_threshold = 0.02
    last_odom_time = 9.5
    linear_velocity = 0.0
    angular_velocity = 0.0
    last_cmd_vel_time = None
    last_localization_time = 5.0
    localization_frame_id = 'map'
    localization_position = (1.25, -0.5)
    last_marker_ids_time = 9.5
    marker_ids = []

    robot_is_stationary = AmrSystemMonitor.robot_is_stationary
    motion_values = AmrSystemMonitor.motion_values

    def make_status(self, name, level, message, values):
        return AmrSystemMonitor.make_status(
            self,
            name,
            level,
            message,
            values,
        )


def level_value(status):
    level = status.level
    if isinstance(level, (bytes, bytearray, memoryview)):
        return bytes(level)[0]
    return int(level)


def values_by_key(status):
    return {
        item.key: item.value
        for item in status.values
    }


def test_stationary_robot_accepts_missing_velocity_command():
    monitor = StubMonitor()

    status = AmrSystemMonitor.command_status(monitor, 10.0)

    assert level_value(status) == 0
    assert status.message == (
        'Robot stationary; no active velocity command'
    )


def test_moving_robot_warns_about_stale_velocity_command():
    monitor = StubMonitor()
    monitor.linear_velocity = 0.2
    monitor.last_cmd_vel_time = 5.0

    status = AmrSystemMonitor.command_status(monitor, 10.0)

    assert level_value(status) == 1
    assert status.message == (
        'Robot moving; velocity command is stale'
    )


def test_stationary_robot_accepts_aged_amcl_pose():
    monitor = StubMonitor()

    status = AmrSystemMonitor.localization_status(
        monitor,
        10.0,
    )

    assert level_value(status) == 0
    assert status.message == (
        'Robot stationary; last AMCL pose remains valid'
    )
    assert values_by_key(status)['age_seconds'] == '5.00'


def test_moving_robot_warns_about_aged_amcl_pose():
    monitor = StubMonitor()
    monitor.angular_velocity = 0.2

    status = AmrSystemMonitor.localization_status(
        monitor,
        10.0,
    )

    assert level_value(status) == 1
    assert status.message == (
        'AMCL pose is stale while robot is moving'
    )


def test_no_visible_aruco_marker_is_normal():
    monitor = StubMonitor()

    status = AmrSystemMonitor.aruco_status(monitor, 10.0)

    assert level_value(status) == 0
    assert status.message == 'No marker currently visible'
