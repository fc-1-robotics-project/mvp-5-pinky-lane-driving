"""Unit tests for the fail-closed velocity gate policy."""

from pinky_fleet_safety.gate_policy import LED_HOLD, LED_RUN, LED_STOP
from pinky_fleet_safety.gate_policy import led_state_for_gate
from pinky_fleet_safety.gate_policy import MODE_ESTOP, MODE_HOLD, MODE_RUN
from pinky_fleet_safety.gate_policy import VelocityGatePolicy


def make_policy() -> VelocityGatePolicy:
    """Return a policy with short deterministic timeouts."""
    return VelocityGatePolicy('robot1', 0.25, 1.0)


def permit(policy, mode=MODE_RUN, now=10.0, sequence=1, ttl=0.5):
    """Submit a valid test permit."""
    return policy.accept_permit(
        robot_id='robot1',
        controller_id='controller-session-a',
        sequence=sequence,
        mode=mode,
        ttl_sec=ttl,
        lease_id='lease-a',
        now=now,
    )


def test_gate_starts_fail_closed() -> None:
    """No output is allowed before the first permit."""
    decision = make_policy().evaluate(10.0)
    assert not decision.output_enabled
    assert decision.status == 'NO_PERMIT'


def test_run_requires_fresh_permit_and_command() -> None:
    """A fresh RUN permit and command enable output."""
    policy = make_policy()
    assert permit(policy)
    policy.note_command(10.1)
    assert policy.evaluate(10.2).output_enabled
    assert policy.evaluate(10.36).status == 'COMMAND_TIMEOUT'
    assert policy.evaluate(10.51).status == 'PERMIT_TIMEOUT'


def test_hold_and_estop_block_output() -> None:
    """Both non-running permit modes force a stop."""
    policy = make_policy()
    policy.note_command(10.0)
    assert permit(policy, mode=MODE_HOLD)
    assert policy.evaluate(10.1).status == 'HOLD'
    assert permit(policy, mode=MODE_ESTOP, sequence=2)
    assert policy.evaluate(10.1).status == 'E_STOP'


def test_wrong_robot_and_older_sequence_are_rejected() -> None:
    """A permit cannot target another robot or roll state backward."""
    policy = make_policy()
    assert not policy.accept_permit(
        robot_id='robot2',
        controller_id='controller-session-a',
        sequence=1,
        mode=MODE_RUN,
        ttl_sec=0.5,
        lease_id='',
        now=10.0,
    )
    assert permit(policy, sequence=4)
    assert not permit(policy, sequence=3)
    assert permit(policy, sequence=4, now=10.1)


def test_new_controller_session_may_restart_sequence() -> None:
    """A fleet-manager restart can establish a new sequence space."""
    policy = make_policy()
    assert permit(policy, sequence=100)
    assert policy.accept_permit(
        robot_id='robot1',
        controller_id='controller-session-b',
        sequence=1,
        mode=MODE_HOLD,
        ttl_sec=0.5,
        lease_id='',
        now=11.0,
    )


def test_led_state_follows_permit_mode_not_command_freshness() -> None:
    """LED colours represent permission rather than command activity."""
    assert led_state_for_gate(MODE_RUN, True) == LED_RUN
    assert led_state_for_gate(MODE_HOLD, True) == LED_HOLD
    assert led_state_for_gate(MODE_ESTOP, True) == LED_STOP
    assert led_state_for_gate(MODE_RUN, False) == LED_STOP
