from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class EpisodeSummary:
    episode: int
    start_phase: str
    obs_shapes: dict[str, tuple[int, ...]] = field(default_factory=dict)
    steps: int = 0
    total_return: float = 0.0
    final_phase: str = ""
    final_command: str | None = None
    final_executor_status: str = ""
    invalid_command_count: int = 0
    phase_success_count: int = 0
    phase_failure_count: int = 0
    ik_fail_count: int = 0
    workspace_fail_count: int = 0
    recovery_count: int = 0
    drop_count: int = 0
    timeout_count: int = 0
    max_attempts_exceeded_count: int = 0
    max_attempt_count_seen: int = 0
    rendered_frames: int = 0
    phase_counts: dict[str, int] = field(default_factory=dict)
    command_counts: dict[str, int] = field(default_factory=dict)
    executor_status_counts: dict[str, int] = field(default_factory=dict)
    planner_fail_reason_counts: dict[str, int] = field(default_factory=dict)
    planner_fail_class_counts: dict[str, int] = field(default_factory=dict)
    phase_transition_counts: dict[str, int] = field(default_factory=dict)
    reward_component_sums: dict[str, float] = field(default_factory=dict)

    def record_step(self, reward: float, info: dict) -> None:
        self.steps += 1
        self.total_return += float(reward)
        self.final_phase = str(info.get("phase", ""))
        self.final_command = info.get("command")
        self.final_executor_status = str(info.get("executor_status", ""))
        phase = self.final_phase
        self.phase_counts[phase] = self.phase_counts.get(phase, 0) + 1
        command = str(info.get("command", ""))
        if command:
            self.command_counts[command] = self.command_counts.get(command, 0) + 1
        status = str(info.get("executor_status", ""))
        if status:
            self.executor_status_counts[status] = self.executor_status_counts.get(status, 0) + 1
        transition = f"{info.get('phase_before', '')}->{info.get('phase_after', info.get('phase', ''))}"
        self.phase_transition_counts[transition] = self.phase_transition_counts.get(transition, 0) + 1
        if not info.get("valid_command", True):
            self.invalid_command_count += 1
        if info.get("phase_success", False):
            self.phase_success_count += 1
        if info.get("phase_failure", False):
            self.phase_failure_count += 1
        if info.get("executor_status") == "IK_FAIL":
            self.ik_fail_count += 1
        if info.get("executor_status") == "WORKSPACE_FAIL":
            self.workspace_fail_count += 1
        if info.get("executor_status") == "RECOVERED":
            self.recovery_count += 1
        if info.get("dropped", False):
            self.drop_count += 1
        if "timeout" in info.get("reward_components", {}):
            self.timeout_count += 1
        if info.get("max_attempts_exceeded", False):
            self.max_attempts_exceeded_count += 1
        self.max_attempt_count_seen = max(
            self.max_attempt_count_seen,
            int(info.get("attempt_count", 0)),
        )
        fail_reason = str(info.get("planner_fail_reason", ""))
        if fail_reason:
            self.planner_fail_reason_counts[fail_reason] = (
                self.planner_fail_reason_counts.get(fail_reason, 0) + 1
            )
        fail_class = str(info.get("planner_fail_class", ""))
        if fail_class:
            self.planner_fail_class_counts[fail_class] = (
                self.planner_fail_class_counts.get(fail_class, 0) + 1
            )
        for key, value in info.get("reward_components", {}).items():
            self.reward_component_sums[key] = self.reward_component_sums.get(key, 0.0) + float(value)

    def to_dict(self) -> dict:
        return {
            "episode": self.episode,
            "start_phase": self.start_phase,
            "obs_shapes": {key: list(value) for key, value in self.obs_shapes.items()},
            "steps": self.steps,
            "return": self.total_return,
            "final_phase": self.final_phase or self.start_phase,
            "final_command": self.final_command,
            "final_executor_status": self.final_executor_status,
            "invalid_command_count": self.invalid_command_count,
            "phase_success_count": self.phase_success_count,
            "phase_failure_count": self.phase_failure_count,
            "ik_fail_count": self.ik_fail_count,
            "workspace_fail_count": self.workspace_fail_count,
            "recovery_count": self.recovery_count,
            "drop_count": self.drop_count,
            "timeout_count": self.timeout_count,
            "max_attempts_exceeded_count": self.max_attempts_exceeded_count,
            "max_attempt_count_seen": self.max_attempt_count_seen,
            "rendered_frames": self.rendered_frames,
            "phase_counts": self.phase_counts,
            "command_counts": self.command_counts,
            "executor_status_counts": self.executor_status_counts,
            "planner_fail_reason_counts": self.planner_fail_reason_counts,
            "planner_fail_class_counts": self.planner_fail_class_counts,
            "phase_transition_counts": self.phase_transition_counts,
            "reward_component_sums": self.reward_component_sums,
        }
