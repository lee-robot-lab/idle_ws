from __future__ import annotations

from enum import IntEnum


class Phase(IntEnum):
    OBSERVE_OBJECT = 0
    MOVE_TO_PREGRASP = 1
    GRASP = 2
    LIFT = 3
    MOVE_TO_PLACE = 4
    PLACE = 5
    RETREAT = 6
    DONE = 7
    FAILURE = 8


class Command(IntEnum):
    MOVE_TO_PREGRASP = 0
    GRASP = 1
    LIFT = 2
    MOVE_TO_PLACE = 3
    PLACE = 4
    HOME = 5
    RECOVERY = 6
    STOP = 7


class StepResult(IntEnum):
    NONE = 0
    SUCCESS = 1
    FAILURE = 2
    INVALID = 3
    TIMEOUT = 4


PHASE_COUNT = len(Phase)
COMMAND_COUNT = len(Command)
POLICY_COMMAND_COUNT = COMMAND_COUNT - 1  # STOP is terminal/safety state, not a learned PPO action.
RESULT_COUNT = len(StepResult)


ALLOWED_COMMANDS: dict[Phase, set[Command]] = {
    Phase.OBSERVE_OBJECT: {Command.MOVE_TO_PREGRASP},
    Phase.MOVE_TO_PREGRASP: {
        Command.MOVE_TO_PREGRASP,
        Command.GRASP,
        Command.RECOVERY,
    },
    Phase.GRASP: {Command.GRASP, Command.LIFT, Command.RECOVERY},
    Phase.LIFT: {Command.LIFT, Command.MOVE_TO_PLACE, Command.RECOVERY},
    Phase.MOVE_TO_PLACE: {
        Command.MOVE_TO_PLACE,
        Command.PLACE,
        Command.RECOVERY,
    },
    Phase.PLACE: {Command.PLACE, Command.HOME, Command.RECOVERY},
    Phase.RETREAT: {Command.HOME},
    Phase.DONE: {Command.STOP},
    Phase.FAILURE: {Command.STOP},
}


class PhaseManager:
    def __init__(self) -> None:
        self.phase = Phase.OBSERVE_OBJECT
        self.time_in_phase = 0.0
        self.attempt_count = 0

    def reset(self) -> None:
        self.phase = Phase.OBSERVE_OBJECT
        self.time_in_phase = 0.0
        self.attempt_count = 0

    def is_command_valid(self, command: Command) -> bool:
        return command in ALLOWED_COMMANDS[self.phase]

    def tick(self) -> None:
        self.time_in_phase += 1.0

    def record_success(self) -> None:
        self.attempt_count = 0

    def record_failure(self) -> None:
        self.attempt_count += 1

    def set_phase(self, phase: Phase) -> None:
        if phase != self.phase:
            self.phase = phase
            self.time_in_phase = 0.0
            self.attempt_count = 0
