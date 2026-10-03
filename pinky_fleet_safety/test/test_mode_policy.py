"""Unit tests for mutually exclusive robot-local velocity selection."""

from pinky_fleet_safety.mode_policy import DriveModePolicy


def command(value=0.1):
    """Return one six-axis velocity tuple."""
    return (value, 0.0, 0.0, 0.0, 0.0, 0.0)


def test_policy_starts_stopped() -> None:
    policy = DriveModePolicy(0.25)
    assert policy.evaluate(1.0).status == 'MODE_STOP'


def test_transition_requires_a_new_command_from_selected_source() -> None:
    policy = DriveModePolicy(0.25)
    assert policy.note_command('NAV2', command(), 1.0)
    policy.request_mode('NAV2', 1.1)
    assert policy.evaluate(1.11).status == 'WAITING_FOR_FRESH_SOURCE'
    assert policy.note_command('NAV2', command(), 1.12)
    assert policy.evaluate(1.2).output_enabled
    assert policy.evaluate(1.38).status == 'SOURCE_TIMEOUT'


def test_manual_toggle_restores_lane_mode() -> None:
    policy = DriveModePolicy(0.25)
    policy.request_mode('LANE', 1.0)
    assert policy.request_mode('TOGGLE_MANUAL', 1.1) == 'MANUAL'
    assert policy.request_mode('TOGGLE_MANUAL', 1.2) == 'LANE'


def test_autonomous_request_does_not_steal_manual_control() -> None:
    policy = DriveModePolicy(0.25)
    policy.request_mode('NAV2', 1.0)
    policy.request_mode('TOGGLE_MANUAL', 1.1)
    assert policy.request_mode('LANE', 1.2) == 'MANUAL'
    assert policy.request_mode('TOGGLE_MANUAL', 1.3) == 'LANE'


def test_unselected_sources_never_reach_output() -> None:
    policy = DriveModePolicy(0.25)
    policy.request_mode('NAV2', 1.0)
    assert policy.note_command('LANE', command(0.2), 1.1)
    assert policy.evaluate(1.2).status == 'SOURCE_UNAVAILABLE'
