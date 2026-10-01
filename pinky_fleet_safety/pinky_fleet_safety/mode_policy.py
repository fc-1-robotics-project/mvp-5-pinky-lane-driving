"""Pure policy for selecting exactly one robot-local velocity source."""

from dataclasses import dataclass
import math


MODE_STOP = 'STOP'
MODE_NAV2 = 'NAV2'
MODE_LANE = 'LANE'
MODE_MANUAL = 'MANUAL'
MODE_TOGGLE_MANUAL = 'TOGGLE_MANUAL'
DRIVE_MODES = (MODE_STOP, MODE_NAV2, MODE_LANE, MODE_MANUAL)


@dataclass(frozen=True)
class ModeDecision:
    """Result of one source-selection evaluation."""

    mode: str
    source: str
    output_enabled: bool
    status: str
    velocity: tuple


class DriveModePolicy:
    """Fail closed while switching among Nav2, lane, and manual commands."""

    def __init__(self, command_timeout: float) -> None:
        if not math.isfinite(command_timeout) or command_timeout <= 0.0:
            raise ValueError('command_timeout must be finite and positive')
        self.command_timeout = command_timeout
        self.mode = MODE_STOP
        self.previous_autonomous_mode = MODE_NAV2
        self.activated_at = 0.0
        self.commands = {}

    def request_mode(self, requested_mode: str, now: float) -> str:
        """Apply a mode request and return the resulting active mode."""
        if not math.isfinite(now) or now < 0.0:
            raise ValueError('now must be finite and nonnegative')
        requested = str(requested_mode).strip().upper()
        toggling_manual = requested == MODE_TOGGLE_MANUAL
        if toggling_manual:
            requested = (
                self.previous_autonomous_mode
                if self.mode == MODE_MANUAL
                else MODE_MANUAL
            )
        if requested not in DRIVE_MODES:
            raise ValueError(f'unknown drive mode: {requested_mode}')
        if (not toggling_manual and self.mode == MODE_MANUAL
                and requested in (MODE_NAV2, MODE_LANE)):
            # Autonomous controllers may advance while the operator temporarily
            # owns the robot. Remember their latest mode without stealing manual.
            self.previous_autonomous_mode = requested
            return self.mode
        if requested == self.mode:
            return self.mode
        if requested in (MODE_NAV2, MODE_LANE):
            self.previous_autonomous_mode = requested
        elif requested == MODE_MANUAL and self.mode in (MODE_NAV2, MODE_LANE):
            self.previous_autonomous_mode = self.mode
        self.mode = requested
        self.activated_at = now
        return self.mode

    def note_command(self, source: str, velocity: tuple, now: float) -> bool:
        """Record a finite command for one known source."""
        source = str(source).strip().upper()
        if source not in (MODE_NAV2, MODE_LANE, MODE_MANUAL):
            return False
        if (len(velocity) != 6 or not all(math.isfinite(v) for v in velocity)
                or not math.isfinite(now) or now < 0.0):
            return False
        self.commands[source] = (tuple(float(v) for v in velocity), now)
        return True

    def evaluate(self, now: float) -> ModeDecision:
        """Select a fresh post-transition command or return an explicit stop."""
        stopped = (0.0,) * 6
        if not math.isfinite(now) or now < 0.0:
            self.mode = MODE_STOP
            return ModeDecision(self.mode, '', False, 'CLOCK_FAILURE', stopped)
        if self.mode == MODE_STOP:
            return ModeDecision(self.mode, '', False, 'MODE_STOP', stopped)
        command = self.commands.get(self.mode)
        if command is None:
            return ModeDecision(
                self.mode, self.mode, False, 'SOURCE_UNAVAILABLE', stopped,
            )
        velocity, received_at = command
        if received_at < self.activated_at:
            return ModeDecision(
                self.mode, self.mode, False, 'WAITING_FOR_FRESH_SOURCE', stopped,
            )
        age = now - received_at
        if age < 0.0 or age > self.command_timeout:
            return ModeDecision(
                self.mode, self.mode, False, 'SOURCE_TIMEOUT', stopped,
            )
        return ModeDecision(self.mode, self.mode, True, 'ACTIVE', velocity)
