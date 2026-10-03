"""ROS-independent fail-closed policy for the robot velocity gate."""

from dataclasses import dataclass
from typing import Optional


MODE_HOLD = 0
MODE_RUN = 1
MODE_ESTOP = 2
VALID_MODES = (MODE_HOLD, MODE_RUN, MODE_ESTOP)
LED_RUN = 'RUN'
LED_HOLD = 'HOLD'
LED_STOP = 'STOP'


def led_state_for_gate(mode: int, permit_fresh: bool) -> str:
    """Return the fail-safe LED state for a local gate decision."""
    if not permit_fresh or mode == MODE_ESTOP:
        return LED_STOP
    if mode == MODE_HOLD:
        return LED_HOLD
    if mode == MODE_RUN:
        return LED_RUN
    return LED_STOP


@dataclass(frozen=True)
class GateDecision:
    """Result of evaluating permit and velocity-command freshness."""

    output_enabled: bool
    permit_fresh: bool
    command_fresh: bool
    status: str


class VelocityGatePolicy:
    """Track short-lived fleet permits using only local monotonic time."""

    def __init__(
        self,
        robot_id: str,
        command_timeout_sec: float,
        max_permit_ttl_sec: float,
    ) -> None:
        if not robot_id:
            raise ValueError('robot_id must not be empty')
        if command_timeout_sec <= 0.0:
            raise ValueError('command_timeout_sec must be greater than zero')
        if max_permit_ttl_sec <= 0.0:
            raise ValueError('max_permit_ttl_sec must be greater than zero')

        self.robot_id = robot_id
        self.command_timeout_sec = command_timeout_sec
        self.max_permit_ttl_sec = max_permit_ttl_sec
        self.mode = MODE_HOLD
        self.controller_id = ''
        self.last_permit_sequence = 0
        self.active_lease_id = ''
        self.permit_expires_at: Optional[float] = None
        self.last_command_at: Optional[float] = None

    def accept_permit(
        self,
        *,
        robot_id: str,
        controller_id: str,
        sequence: int,
        mode: int,
        ttl_sec: float,
        lease_id: str,
        now: float,
    ) -> bool:
        """Accept a valid permit, rejecting another robot or stale update."""
        if robot_id != self.robot_id or not controller_id:
            return False
        if mode not in VALID_MODES or ttl_sec <= 0.0:
            return False
        if controller_id == self.controller_id:
            if sequence < self.last_permit_sequence:
                return False

        self.controller_id = controller_id
        self.last_permit_sequence = sequence
        self.mode = mode
        self.active_lease_id = lease_id
        effective_ttl = min(ttl_sec, self.max_permit_ttl_sec)
        self.permit_expires_at = now + effective_ttl
        return True

    def note_command(self, now: float) -> None:
        """Record receipt of a candidate velocity command."""
        self.last_command_at = now

    def evaluate(self, now: float) -> GateDecision:
        """Return whether the most recent candidate command may pass."""
        permit_fresh = (
            self.permit_expires_at is not None
            and now < self.permit_expires_at
        )
        command_fresh = (
            self.last_command_at is not None
            and now - self.last_command_at < self.command_timeout_sec
        )

        if not permit_fresh:
            status = (
                'NO_PERMIT'
                if self.permit_expires_at is None
                else 'PERMIT_TIMEOUT'
            )
        elif self.mode == MODE_ESTOP:
            status = 'E_STOP'
        elif self.mode == MODE_HOLD:
            status = 'HOLD'
        elif not command_fresh:
            status = 'COMMAND_TIMEOUT'
        else:
            return GateDecision(True, True, True, 'RUN')

        return GateDecision(False, permit_fresh, command_fresh, status)
