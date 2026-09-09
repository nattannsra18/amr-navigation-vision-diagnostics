"""Validate declared robot capabilities against observed ROS interfaces."""

from __future__ import annotations

from typing import Any


KNOWN_CAPABILITIES = frozenset({
    'diagnostics',
    'localization',
    'mapping',
    'navigation',
})


def validate_robot_profile(
    capabilities: list[str],
    observation: dict[str, Any],
    *,
    freshness_seconds: float = 3.0,
) -> dict[str, Any]:
    """Build a serializable readiness report from ROS observations."""
    declared = {item.strip().lower() for item in capabilities if item.strip()}
    results: list[dict[str, str]] = []

    def add(
        check_id: str,
        category: str,
        passed: bool,
        message: str,
        *,
        observed: str | None = None,
        warning: bool = False,
    ) -> None:
        item = {
            'check_id': check_id,
            'category': category,
            'status': 'PASS' if passed else ('WARN' if warning else 'FAIL'),
            'message': message,
        }
        if observed is not None:
            item['observed'] = observed
        results.append(item)

    def interface(
        check_id: str,
        capability: str,
        key: str,
        label: str,
    ) -> None:
        if capability not in declared:
            return
        available = bool(observation.get(key))
        add(
            check_id,
            'INTERFACE',
            available,
            (
                f'{label} is available'
                if available else f'{label} is unavailable'
            ),
            observed=str(observation.get(f'{key}_name') or label),
        )

    def fresh(
        check_id: str,
        capability: str,
        age_key: str,
        label: str,
    ) -> None:
        if capability not in declared:
            return
        age = observation.get(age_key)
        passed = (
            isinstance(age, (int, float))
            and 0 <= float(age) <= freshness_seconds
        )
        observed = (
            'never received' if age is None else f'{float(age):.2f} s old'
        )
        add(
            check_id,
            'DATA',
            passed,
            f'{label} is fresh' if passed else f'{label} has no fresh data',
            observed=observed,
        )

    for unknown in sorted(declared - KNOWN_CAPABILITIES):
        add(
            f'capability.{unknown}',
            'CAPABILITY',
            False,
            f'Capability {unknown} has no validator contract',
            observed=unknown,
            warning=True,
        )

    interface(
        'interface.navigate_action',
        'navigation',
        'navigate_action',
        'NavigateToPose action',
    )
    interface(
        'interface.compute_path_action',
        'navigation',
        'compute_path_action',
        'ComputePathToPose action',
    )
    fresh('data.odom', 'navigation', 'odom_age_seconds', 'Odometry topic')
    fresh(
        'data.amcl_pose',
        'localization',
        'amcl_pose_age_seconds',
        'AMCL pose topic',
    )

    if declared & {'navigation', 'localization'}:
        for check_id, key, label in (
            ('tf.map_to_odom', 'tf_map_to_odom', 'TF map to odom'),
            ('tf.odom_to_base', 'tf_odom_to_base', 'TF odom to base_link'),
        ):
            available = bool(observation.get(key))
            add(
                check_id,
                'TF',
                available,
                (
                    f'{label} is connected'
                    if available else f'{label} is unavailable'
                ),
            )

    if 'localization' in declared:
        active = observation.get('amcl_state') == 'ACTIVE'
        add(
            'lifecycle.amcl',
            'LIFECYCLE',
            active,
            (
                'AMCL lifecycle is active'
                if active else 'AMCL lifecycle is not active'
            ),
            observed=str(observation.get('amcl_state') or 'UNKNOWN'),
        )
        interface(
            'interface.global_localization_service',
            'localization',
            'global_localization_service',
            'Global localization service',
        )

    if 'mapping' in declared:
        configured = bool(observation.get('maps_directory'))
        add(
            'capability.mapping_storage',
            'CAPABILITY',
            configured,
            (
                'Writable map storage is configured'
                if configured else 'Map storage directory is not configured'
            ),
            observed=str(
                observation.get('maps_directory') or 'not configured'
            ),
        )
        interface(
            'interface.load_map_service',
            'mapping',
            'load_map_service',
            'LoadMap service',
        )

    if 'diagnostics' in declared:
        fresh(
            'data.diagnostics',
            'diagnostics',
            'diagnostics_age_seconds',
            'Diagnostics topic',
        )

    map_received = observation.get('map_age_seconds') is not None
    if declared & {'navigation', 'localization', 'mapping'}:
        add(
            'data.map',
            'DATA',
            map_received,
            (
                'Occupancy map has been received'
                if map_received else 'Occupancy map has not been received'
            ),
            observed=(
                'never received'
                if not map_received
                else (
                    f"{float(observation['map_age_seconds']):.2f} s "
                    'since update'
                )
            ),
        )

    failures = sum(item['status'] == 'FAIL' for item in results)
    warnings = sum(item['status'] == 'WARN' for item in results)
    status = 'NOT_READY' if failures else ('DEGRADED' if warnings else 'READY')
    if failures:
        detail = f'{failures} required profile checks failed'
    elif warnings:
        detail = f'{warnings} profile checks need attention'
    else:
        detail = f'All {len(results)} declared profile checks passed'
    return {
        'status': status,
        'detail': detail,
        'checks': results,
    }
