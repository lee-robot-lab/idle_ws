from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import mujoco
import numpy as np


@dataclass
class JointTrajectory:
    q: np.ndarray
    qd: np.ndarray
    duration: float


def make_joint_space_trajectory(
    q_start: np.ndarray,
    q_goal: np.ndarray,
    control_hz: float = 100.0,
    q_speed: float = 0.8,
    min_duration: float = 0.4,
    max_duration: float = 2.5,
) -> JointTrajectory:
    q_start = np.asarray(q_start, dtype=np.float64)
    q_goal = np.asarray(q_goal, dtype=np.float64)
    max_delta = float(np.max(np.abs(q_goal - q_start))) if q_start.size else 0.0
    duration = float(np.clip(max_delta / max(q_speed, 1e-6), min_duration, max_duration))
    steps = max(2, int(round(duration * control_hz)))
    alpha = np.linspace(0.0, 1.0, steps)[:, None]
    q = (1.0 - alpha) * q_start[None, :] + alpha * q_goal[None, :]
    qd = np.gradient(q, duration / max(steps - 1, 1), axis=0)
    return JointTrajectory(q=q, qd=qd, duration=duration)


class PdJointTrajectoryExecutor:
    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        names,
        arm_kp: float = 120.0,
        arm_kd: float = 8.0,
        gripper_kp: float = 80.0,
        gripper_kd: float = 4.0,
        gravity_comp_scale: float = 1.0,
    ) -> None:
        self.model = model
        self.data = data
        self.names = names
        self.arm_kp = arm_kp
        self.arm_kd = arm_kd
        self.gripper_kp = gripper_kp
        self.gripper_kd = gripper_kd
        self.gravity_comp_scale = float(gravity_comp_scale)

    def execute(
        self,
        trajectory: JointTrajectory,
        gripper_target: float | None = None,
        settle_steps: int = 80,
        post_step: Callable[[], None] | None = None,
    ) -> int:
        sim_steps = 0
        if gripper_target is None:
            gripper_target = float(self.data.qpos[self.names.finger_r_qposadr])

        for q_des, qd_des in zip(trajectory.q, trajectory.qd):
            self._step_pd(q_des, qd_des, gripper_target)
            if post_step is not None:
                post_step()
            sim_steps += 1

        final_q = trajectory.q[-1]
        zero_qd = np.zeros_like(final_q)
        for _ in range(max(0, settle_steps)):
            self._step_pd(final_q, zero_qd, gripper_target)
            if post_step is not None:
                post_step()
            sim_steps += 1
        return sim_steps

    def _step_pd(self, q_des: np.ndarray, qd_des: np.ndarray, gripper_target: float) -> None:
        ctrl = np.zeros(self.model.nu, dtype=np.float64)

        arm_q = self.data.qpos[self.names.arm_qposadr]
        arm_qd = self.data.qvel[self.names.arm_dofadr]
        arm_tau = self.arm_kp * (q_des - arm_q) + self.arm_kd * (qd_des - arm_qd)
        arm_tau = arm_tau + self.gravity_comp_scale * self.data.qfrc_bias[self.names.arm_dofadr]
        arm_limits = self.model.actuator_ctrlrange[self.names.arm_actuator_ids]
        ctrl[self.names.arm_actuator_ids] = np.clip(arm_tau, arm_limits[:, 0], arm_limits[:, 1])

        gripper_q = self.data.qpos[self.names.finger_r_qposadr]
        gripper_qd = self.data.qvel[self.names.finger_r_dofadr]
        gripper_tau = self.gripper_kp * (gripper_target - gripper_q) - self.gripper_kd * gripper_qd
        gripper_limits = self.model.actuator_ctrlrange[self.names.gripper_actuator_id]
        ctrl[self.names.gripper_actuator_id] = float(np.clip(gripper_tau, gripper_limits[0], gripper_limits[1]))

        self.data.ctrl[:] = ctrl
        mujoco.mj_step(self.model, self.data)
