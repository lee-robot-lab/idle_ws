from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from mujoco_phase_rl.controllers.dls_ik import DlsIkSolver
from mujoco_phase_rl.controllers.feasibility import FeasibilityChecker
from mujoco_phase_rl.controllers.trajectory_controller import (
    PdJointTrajectoryExecutor,
    make_joint_space_trajectory,
)
from mujoco_phase_rl.perception.image_embedding import SlotEmbedder, IMAGE_EMBEDDING_SIZE
from mujoco_phase_rl.perception.pose_provider import (
    PoseEstimate,
    SlotState,
    SlotStateBridge,
    make_pose_provider,
)
from mujoco_phase_rl.perception.snapshot_observer import SnapshotObserver, SnapshotState
from mujoco_phase_rl.tasks.phase_manager import (
    ALLOWED_COMMANDS,
    COMMAND_COUNT,
    Command,
    Phase,
    PhaseManager,
    StepResult,
)
from mujoco_phase_rl.tasks.pick_place_task import PickPlaceTask, TaskSample, BLOCK_COLORS
from mujoco_phase_rl.tasks.reward import FAILURE_STATUSES, compute_phase_reward
from mujoco_phase_rl.utils.mujoco_loader import load_task_scene, set_freejoint_pose
from mujoco_phase_rl.utils.spaces import gym, spaces
from mujoco_phase_rl.world_model.phase_destination import encode_phase_destination_2d


GRASP_TARGET_Z_DELTA = 0.018
GRASP_DZ_ACTION_SCALE = 0.20


@dataclass
class DecodedAction:
    command: Command
    raw_command: Command
    command_was_masked: bool
    delta_xyz: np.ndarray
    dyaw: float
    gripper_close: bool
    lift_height: float


class PhasePickPlaceEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 30}

    def __init__(
        self,
        robot_xml_path: str | None = None,
        render_mode: str | None = None,
        max_episode_steps: int = 100,
        frame_skip: int = 25,
        mask_invalid_commands: bool = False,
        image_embedding_mode: str = "zeros",
        image_width: int = 64,
        image_height: int = 64,
        image_embedding_interval: int = 1,
        pose_source: str = "gt",
        pose_noise_std: float = 0.0,
        target_noise_std: float = 0.0,
        pose_dropout_prob: float = 0.0,
        max_phase_failures: int = 8,
        show_target_marker: bool = True,
        work_surface_rgba: str = "0.42 0.52 0.53 0.65",
        slot_stage1_ckpt: str | None = None,
        slot_diff_ckpt: str | None = None,
        slot_color_net_ckpt: str | None = None,
        slot_device: str = "cpu",
        slot_transition_ckpt: str | None = None,
        perturb_prob: float = 0.0,
        perturb_max_m: float = 0.08,
        stack_prob: float = 0.6,
    ) -> None:
        super().__init__()
        self.render_mode = render_mode
        self.max_episode_steps = int(max_episode_steps)
        self.frame_skip = int(frame_skip)
        self.mask_invalid_commands = bool(mask_invalid_commands)
        self.image_embedding_mode = image_embedding_mode
        if self.image_embedding_mode not in {"zeros", "slot"}:
            raise ValueError("image_embedding_mode must be one of: zeros, slot")
        self.image_width = int(image_width)
        self.image_height = int(image_height)
        self.image_embedding_interval = max(1, int(image_embedding_interval))
        self.pose_source = pose_source
        self.pose_noise_std = max(0.0, float(pose_noise_std))
        self.target_noise_std = max(0.0, float(target_noise_std))
        self.pose_dropout_prob = float(np.clip(pose_dropout_prob, 0.0, 1.0))
        self.max_phase_failures = max(1, int(max_phase_failures))
        self.show_target_marker = bool(show_target_marker)
        self.work_surface_rgba = str(work_surface_rgba)
        self.scene = load_task_scene(
            robot_xml_path,
            show_target_marker=self.show_target_marker,
            work_surface_rgba=self.work_surface_rgba,
        )
        self.model = self.scene.model
        self.data = self.scene.data
        self.names = self.scene.names
        self.task = PickPlaceTask(stack_prob=stack_prob)
        self._pick_color: str = "red"
        self._object_body_id: int = self.names.block_body_ids["red"]
        self._target_block_body_id: int | None = None
        self.phase_manager = PhaseManager()
        self.observer = SnapshotObserver(self.model, self.data, self.names)
        self.pose_provider = make_pose_provider(
            self.pose_source,
            self.model,
            self.data,
            self.names,
            object_pos_noise_std=self.pose_noise_std,
            target_pos_noise_std=self.target_noise_std,
            dropout_prob=self.pose_dropout_prob,
        )
        self.feasibility = FeasibilityChecker()
        self.ik = DlsIkSolver(
            self.model,
            self.data,
            self.names.arm_qposadr,
            self.names.arm_dofadr,
            self.names.joint_ranges[:6],
            self.names.ee_site_id,
        )
        self.trajectory_executor = PdJointTrajectoryExecutor(self.model, self.data, self.names)
        self.renderer = None
        self.post_mj_step_callback = None
        self.slot_embedder: SlotEmbedder | None = None
        self.slot_state_bridge: SlotStateBridge | None = None
        self._cached_slot_diff_emb: np.ndarray = np.zeros(IMAGE_EMBEDDING_SIZE, dtype=np.float32)
        self._cached_curr_slots: dict | None = None
        self.slot_state_bridge_grounded = False
        # 배치 추론용 deferred 모드 (BatchedSlotDummyVecEnv에서 사용)
        self._slot_embed_deferred: bool = False
        self._slot_embed_pending_img = None
        self._embed_injected: bool = False
        if self.image_embedding_mode == "slot":
            if not (slot_stage1_ckpt and slot_diff_ckpt and slot_color_net_ckpt):
                raise ValueError(
                    "slot mode requires slot_stage1_ckpt, slot_diff_ckpt, slot_color_net_ckpt"
                )
            self.slot_embedder = SlotEmbedder(
                slot_stage1_ckpt,
                slot_diff_ckpt,
                slot_color_net_ckpt,
                device=slot_device,
            )
            self.slot_state_bridge = SlotStateBridge()
        self.last_pose_estimate = None
        self.home_q = np.zeros(6, dtype=np.float64)

        self._wm_model = None
        self._wm_h = None
        if slot_transition_ckpt is not None:
            import torch
            from mujoco_phase_rl.world_model.slot_transition_model import SlotTransitionModel
            ckpt = torch.load(slot_transition_ckpt, map_location="cpu", weights_only=False)
            self._wm_model = SlotTransitionModel(
                input_dim=ckpt.get("input_dim", 84),
                h_dim=ckpt.get("h_dim", 128),
                rssm_latent_dim=ckpt.get("rssm_latent_dim", 64),
            )
            self._wm_model.load_state_dict(ckpt["state_dict"])
            self._wm_model.eval()

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(14,), dtype=np.float32)
        inf = np.inf
        self.observation_space = spaces.Dict(
            {
                "robot": spaces.Box(low=-inf, high=inf, shape=(11,), dtype=np.float32),
                "task": spaces.Box(low=-inf, high=inf, shape=(4,), dtype=np.float32),
                "phase": spaces.Box(low=-inf, high=inf, shape=(9,), dtype=np.float32),
                "history": spaces.Box(low=-inf, high=inf, shape=(13,), dtype=np.float32),
                "slot_diff": spaces.Box(low=-inf, high=inf, shape=(64,), dtype=np.float32),
                "rssm_latent": spaces.Box(low=-inf, high=inf, shape=(64,), dtype=np.float32),
                "cmd": spaces.Box(low=0.0, high=1.0, shape=(9,), dtype=np.float32),
            }
        )

        self.perturb_prob = float(perturb_prob)
        self.perturb_max_m = float(perturb_max_m)

        self.rng = np.random.default_rng()
        self.step_count = 0
        self.prev_command_id: int | None = None
        self.prev_result = StepResult.NONE
        self.prev_reward = 0.0
        self.object_grasped = False
        self.dropped = False
        self.grasp_offset_pos = np.zeros(3, dtype=np.float64)
        self.grasp_object_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.current_task: TaskSample | None = None

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        injected = (options or {}).get("task_sample")
        if seed is not None:
            self.rng = np.random.default_rng(seed)

        mujoco.mj_resetData(self.model, self.data)
        self.phase_manager.reset()
        self.step_count = 0
        self.prev_command_id = None
        self.prev_result = StepResult.NONE
        self.prev_reward = 0.0
        self.object_grasped = False
        self.dropped = False
        self.grasp_offset_pos = np.zeros(3, dtype=np.float64)
        self.grasp_object_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.last_pose_estimate = None
        if self.slot_embedder is not None:
            self.slot_embedder.reset()
        self.slot_state_bridge_grounded = False
        self._cached_slot_diff_emb = np.zeros(IMAGE_EMBEDDING_SIZE, dtype=np.float32)
        self._cached_curr_slots = None
        self.pose_provider.reset()
        if self._wm_model is not None:
            import torch
            self._wm_h = self._wm_model.init_hidden(1, torch.device("cpu"))

        self.current_task = injected if injected is not None else self.task.sample(self.rng)
        self._pick_color = self.current_task.pick_color
        self._object_body_id = self.names.block_body_ids[self._pick_color]
        # PoseProvider가 names.object_body_id를 직접 참조하므로 동기화
        self.names.object_body_id = self._object_body_id
        self._target_block_body_id = (
            self.names.block_body_ids[self.current_task.target_color]
            if self.current_task.target_color is not None else None
        )
        self._apply_home_pose()
        self._apply_task_sample(self.current_task)
        mujoco.mj_forward(self.model, self.data)

        obs = self._observe()
        info = self._info(
            command=None,
            raw_command=None,
            command_was_masked=False,
            valid_command=True,
            executor_status="RESET",
            ik_success=True,
            sim_steps=0,
            reward_components={},
            phase_success=False,
            phase_failure=False,
        )
        return obs, info

    def step(self, action):
        decoded = self._decode_action(action)
        phase_before = self.phase_manager.phase
        valid_command = self._is_command_valid(decoded.command)
        self.step_count += 1
        self.phase_manager.tick()

        sim_steps = 0
        phase_success = False
        phase_failure = False
        timeout = False
        ik_success = True
        extra_info: dict[str, float | int | str | bool] = {}

        if valid_command:
            executor_status, sim_steps, ik_success, phase_success, extra_info = self._execute_command(decoded)
            if phase_success or executor_status == "RECOVERED":
                self.prev_result = StepResult.SUCCESS
            else:
                self.prev_result = StepResult.FAILURE
        else:
            executor_status = "INVALID"
            self.prev_result = StepResult.INVALID

        phase_failure = valid_command and executor_status in FAILURE_STATUSES
        terminated = self.phase_manager.phase in {Phase.DONE, Phase.FAILURE}
        truncated = self.step_count >= self.max_episode_steps and not terminated
        if truncated:
            timeout = True
            self.prev_result = StepResult.TIMEOUT
            extra_info["timeout"] = True

        failed_attempt = (not valid_command) or phase_failure or timeout
        if phase_success:
            self.phase_manager.record_success()
        elif failed_attempt:
            self.phase_manager.record_failure()
            extra_info["attempt_count"] = int(self.phase_manager.attempt_count)
            if self.phase_manager.attempt_count >= self.max_phase_failures:
                extra_info["max_attempts_exceeded"] = True
                extra_info["terminal_failure_reason"] = "max_phase_failures"
                self.phase_manager.set_phase(Phase.FAILURE)
                phase_failure = True

        terminated = self.phase_manager.phase in {Phase.DONE, Phase.FAILURE}
        if terminated and self.phase_manager.phase == Phase.FAILURE:
            truncated = False

        reward, reward_components = compute_phase_reward(
            phase=phase_before,
            command=decoded.command,
            valid_command=valid_command,
            phase_success=phase_success,
            phase_failure=phase_failure,
            dropped=self.dropped,
            timeout=timeout,
            executor_status=executor_status,
            extra_info=extra_info,
        )
        self.prev_reward = reward
        self.prev_command_id = int(decoded.command)

        # mid-episode perturbation
        if self.perturb_prob > 0 and self.rng.random() < self.perturb_prob:
            _BLOCK_BOUNDS = np.array([[-0.15, 0.35], [0.15, 0.45]], dtype=np.float64)
            _BASKET_BOUNDS = np.array([[-0.30, 0.50], [0.30, 0.80]], dtype=np.float64)
            _IDENTITY_QUAT = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

            target = self.rng.choice(["block", "basket"])
            delta = self.rng.uniform(-self.perturb_max_m, self.perturb_max_m, size=2)

            if target == "block" and not self.object_grasped:
                cur = self.data.xpos[self._object_body_id][:2].copy()
                new_xy = np.clip(cur + delta, _BLOCK_BOUNDS[0], _BLOCK_BOUNDS[1])
                new_pos = np.array([new_xy[0], new_xy[1], 0.023], dtype=np.float64)
                set_freejoint_pose(self.data, self.names, new_pos, _IDENTITY_QUAT, color=self._pick_color)
                mujoco.mj_forward(self.model, self.data)
            elif target == "basket":
                cur = self.data.xpos[self.names.basket_body_id][:2].copy()
                new_xy = np.clip(cur + delta, _BASKET_BOUNDS[0], _BASKET_BOUNDS[1])
                self.model.body_pos[self.names.basket_body_id][:2] = new_xy
                mujoco.mj_forward(self.model, self.data)
                if self.current_task is not None:
                    self.current_task.target_pos[:2] = new_xy

        # stack: target block 현재 XY + 동적 z 추적
        if (self.current_task is not None
                and self.current_task.task_type == "stack"
                and self._target_block_body_id is not None):
            tgt_center_z = float(self.data.xpos[self._target_block_body_id][2])
            self.current_task.target_pos[:2] = self.data.xpos[self._target_block_body_id][:2]
            self.current_task.target_pos[2] = tgt_center_z + 0.04  # block full height

        obs = self._observe()
        info = self._info(
            command=decoded.command,
            raw_command=decoded.raw_command,
            command_was_masked=decoded.command_was_masked,
            valid_command=valid_command,
            executor_status=executor_status,
            ik_success=ik_success,
            sim_steps=sim_steps,
            reward_components=reward_components,
            phase_success=phase_success,
            phase_failure=phase_failure,
            extra_info=extra_info,
            phase_before=phase_before,
        )
        return obs, reward, terminated, truncated, info

    def render(self):
        if self.render_mode != "rgb_array":
            return None
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model)
        self.renderer.update_scene(self.data, camera="task_camera")
        return self.renderer.render()

    def close(self) -> None:
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
        if self.slot_embedder is not None:
            self.slot_embedder.close()
            self.slot_embedder = None

    def set_post_mj_step_callback(self, callback) -> None:
        self.post_mj_step_callback = callback

    def _decode_action(self, action) -> DecodedAction:
        arr = np.asarray(action, dtype=np.float32)
        if arr.shape != (14,):
            raise ValueError(f"Expected action shape (14,), got {arr.shape}")
        arr = np.clip(arr, -1.0, 1.0)
        raw_command = Command(int(np.argmax(arr[:COMMAND_COUNT])))
        command = raw_command
        command_was_masked = False
        if self.mask_invalid_commands and not self._is_command_valid(raw_command):
            allowed_commands = sorted(self._allowed_commands_for_current_context(), key=int)
            command = max(allowed_commands, key=lambda candidate: float(arr[int(candidate)]))
            command_was_masked = True
        delta_xyz = np.array([arr[8] * 0.06, arr[9] * 0.06, arr[10] * 0.04], dtype=np.float64)
        dyaw = float(arr[11] * np.deg2rad(30.0))
        gripper_close = bool(arr[12] < 0.0)
        lift_height = float(0.02 + (arr[13] + 1.0) * 0.5 * (0.15 - 0.02))
        return DecodedAction(
            command=command,
            raw_command=raw_command,
            command_was_masked=command_was_masked,
            delta_xyz=delta_xyz,
            dyaw=dyaw,
            gripper_close=gripper_close,
            lift_height=lift_height,
        )

    def _is_command_valid(self, command: Command) -> bool:
        return command in self._allowed_commands_for_current_context()

    def _allowed_commands_for_current_context(self) -> set[Command]:
        allowed_commands = set(ALLOWED_COMMANDS[self.phase_manager.phase])
        if Command.RECOVERY in allowed_commands and not self._recovery_is_context_valid():
            allowed_commands.remove(Command.RECOVERY)
        if (
            self.phase_manager.phase == Phase.PLACE
            and self.object_grasped
            and Command.RECOVERY in allowed_commands
        ):
            allowed_commands.remove(Command.RECOVERY)
        return allowed_commands

    def _recovery_is_context_valid(self) -> bool:
        return (
            self.prev_result in {StepResult.FAILURE, StepResult.TIMEOUT}
            or self.phase_manager.attempt_count > 0
        )

    def _execute_command(
        self,
        decoded: DecodedAction,
    ) -> tuple[str, int, bool, bool, dict[str, float | int | str | bool]]:
        if decoded.command == Command.MOVE_TO_PREGRASP:
            return self._execute_move_to_pregrasp(decoded)
        if decoded.command == Command.GRASP:
            return self._execute_grasp(decoded)
        if decoded.command == Command.LIFT:
            return self._execute_lift(decoded)
        if decoded.command == Command.MOVE_TO_PLACE:
            return self._execute_move_to_place(decoded)
        if decoded.command == Command.PLACE:
            return self._execute_place(decoded)
        if decoded.command == Command.HOME:
            return self._execute_home(decoded)
        if decoded.command == Command.RECOVERY:
            return self._execute_recovery(decoded)
        executor_status, sim_steps = self._execute_skeleton_command(decoded)
        return executor_status, sim_steps, True, False, {}

    def _execute_move_to_pregrasp(
        self,
        decoded: DecodedAction,
    ) -> tuple[str, int, bool, bool, dict[str, float | int | str | bool]]:
        pose = self._estimate_pose()
        perceived_object_pos = pose.object_pos.copy()
        target_pos = perceived_object_pos + np.array([0.0, 0.0, 0.10], dtype=np.float64) + decoded.delta_xyz
        target_yaw = _quat_wxyz_to_yaw(pose.object_quat) + decoded.dyaw
        feasibility = self.feasibility.check_workspace(target_pos)
        if not feasibility.feasible:
            return "WORKSPACE_FAIL", 0, False, False, {
                "target_x": float(target_pos[0]),
                "target_y": float(target_pos[1]),
                "target_z": float(target_pos[2]),
                "failure_reason": feasibility.reason,
            }

        status, sim_steps, ik_success, extra_info = self._move_ee_to_target(
            target_pos,
            gripper_target=self._gripper_open_q(),
            q_speed=0.8,
            settle_steps=80,
            target_yaw=target_yaw,
        )
        if not ik_success:
            return status, sim_steps, False, False, extra_info
        ee_pos = self.data.site_xpos[self.names.ee_site_id].copy()
        object_pos = self.data.xpos[self._object_body_id].copy()
        ee_error = float(np.linalg.norm(target_pos - ee_pos))
        pregrasp_xy_error = float(np.linalg.norm(ee_pos[:2] - object_pos[:2]))
        pregrasp_z_delta = float(ee_pos[2] - object_pos[2])
        phase_success = ee_error <= 0.03 and pregrasp_xy_error <= 0.055 and 0.055 <= pregrasp_z_delta <= 0.145
        extra_info["ee_error"] = ee_error
        extra_info["pregrasp_xy_error"] = pregrasp_xy_error
        extra_info["pregrasp_z_delta"] = pregrasp_z_delta
        if phase_success:
            self.phase_manager.set_phase(Phase.GRASP)
        return "SETTLED" if phase_success else "TARGET_MISS", sim_steps, True, phase_success, extra_info

    def _execute_grasp(
        self,
        decoded: DecodedAction,
    ) -> tuple[str, int, bool, bool, dict[str, float | int | str | bool]]:
        pose = self._estimate_pose()
        perceived_object_pos = pose.object_pos.copy()
        grasp_delta = np.array(
            [
                0.35 * decoded.delta_xyz[0],
                0.35 * decoded.delta_xyz[1],
                GRASP_DZ_ACTION_SCALE * decoded.delta_xyz[2],
            ],
            dtype=np.float64,
        )
        target_pos = (
            perceived_object_pos
            + np.array([0.0, 0.0, GRASP_TARGET_Z_DELTA], dtype=np.float64)
            + grasp_delta
        )
        target_yaw = _quat_wxyz_to_yaw(pose.object_quat) + decoded.dyaw
        feasibility = self.feasibility.check_workspace(target_pos)
        if not feasibility.feasible:
            return "WORKSPACE_FAIL", 0, False, False, {
                "target_x": float(target_pos[0]),
                "target_y": float(target_pos[1]),
                "target_z": float(target_pos[2]),
                "failure_reason": feasibility.reason,
            }

        move_status, move_steps, ik_success, extra_info = self._move_ee_to_target(
            target_pos,
            gripper_target=self._gripper_open_q(),
            q_speed=0.55,
            settle_steps=50,
            target_yaw=target_yaw,
        )
        if not ik_success:
            return move_status, move_steps, False, False, extra_info

        q_hold = self.data.qpos[self.names.arm_qposadr].copy()
        self._run_pd_hold(q_hold, self._gripper_closed_q(), 120)

        ee_pos = self.data.site_xpos[self.names.ee_site_id].copy()
        object_pos = self.data.xpos[self._object_body_id].copy()
        xy_error = float(np.linalg.norm(ee_pos[:2] - object_pos[:2]))
        z_delta = float(ee_pos[2] - object_pos[2])
        distance = float(np.linalg.norm(ee_pos - object_pos))
        finger_q = float(self.data.qpos[self.names.finger_r_qposadr])
        phase_success = (
            xy_error <= 0.045
            and 0.010 <= z_delta <= 0.050
            and finger_q >= self._gripper_grasp_min_q()
        )

        extra_info.update(
            {
                "ee_error": float(np.linalg.norm(target_pos - ee_pos)),
                "grasp_distance": distance,
                "grasp_xy_error": xy_error,
                "grasp_z_delta": z_delta,
                "finger_q": finger_q,
                "finger_open_q": self._gripper_open_q(),
                "finger_grasp_min_q": self._gripper_grasp_min_q(),
                "finger_closed_q": self._gripper_closed_q(),
            }
        )
        if phase_success:
            self.object_grasped = True
            self.grasp_offset_pos = object_pos - ee_pos
            self.grasp_object_quat = self.data.xquat[self._object_body_id].copy()
            self._update_grasped_object_pose()
            self.phase_manager.set_phase(Phase.LIFT)
            return "GRASPED", move_steps + 120, True, True, extra_info

        extra_info["failure_reason"] = "grasp_latch_condition_failed"
        return "GRASP_FAIL", move_steps + 120, True, False, extra_info

    def _execute_lift(
        self,
        decoded: DecodedAction,
    ) -> tuple[str, int, bool, bool, dict[str, float | int | str | bool]]:
        if not self.object_grasped:
            return "NO_GRASP", 0, True, False, {"failure_reason": "object_not_grasped"}

        target_pos = self.data.site_xpos[self.names.ee_site_id].copy()
        target_pos[2] += decoded.lift_height
        feasibility = self.feasibility.check_workspace(target_pos)
        if not feasibility.feasible:
            return "WORKSPACE_FAIL", 0, False, False, {
                "target_x": float(target_pos[0]),
                "target_y": float(target_pos[1]),
                "target_z": float(target_pos[2]),
                "failure_reason": feasibility.reason,
            }

        status, sim_steps, ik_success, extra_info = self._move_ee_to_target(
            target_pos,
            gripper_target=self._gripper_closed_q(),
            q_speed=0.65,
            settle_steps=80,
            target_yaw=self._current_ee_yaw(),
            post_step=self._update_grasped_object_pose,
        )
        if not ik_success:
            return status, sim_steps, False, False, extra_info
        self._update_grasped_object_pose()

        ee_pos = self.data.site_xpos[self.names.ee_site_id].copy()
        object_pos = self.data.xpos[self._object_body_id].copy()
        ee_error = float(np.linalg.norm(target_pos - ee_pos))
        object_z = float(object_pos[2])
        phase_success = self.object_grasped and object_z >= 0.08 and ee_error <= 0.04
        extra_info["ee_error"] = ee_error
        extra_info["object_z"] = object_z
        if phase_success:
            self.phase_manager.set_phase(Phase.MOVE_TO_PLACE)
        return "LIFTED" if phase_success else "LIFT_MISS", sim_steps, True, phase_success, extra_info

    def _execute_move_to_place(
        self,
        decoded: DecodedAction,
    ) -> tuple[str, int, bool, bool, dict[str, float | int | str | bool]]:
        if self.current_task is None:
            return "NO_TASK", 0, True, False, {"failure_reason": "missing_task"}
        if not self.object_grasped:
            return "NO_GRASP", 0, True, False, {"failure_reason": "object_not_grasped"}

        pose = self._estimate_pose()
        target_object_pos = pose.target_pos.copy()
        target_object_pos[2] = 0.14
        target_ee_pos = target_object_pos - self.grasp_offset_pos + decoded.delta_xyz
        target_yaw = pose.target_yaw + decoded.dyaw
        feasibility = self.feasibility.check_workspace(target_ee_pos)
        if not feasibility.feasible:
            return "WORKSPACE_FAIL", 0, False, False, {
                "target_x": float(target_ee_pos[0]),
                "target_y": float(target_ee_pos[1]),
                "target_z": float(target_ee_pos[2]),
                "failure_reason": feasibility.reason,
            }

        status, sim_steps, ik_success, extra_info = self._move_ee_to_target(
            target_ee_pos,
            gripper_target=self._gripper_closed_q(),
            q_speed=0.65,
            settle_steps=80,
            target_yaw=target_yaw,
            post_step=self._update_grasped_object_pose,
        )
        if not ik_success:
            return status, sim_steps, False, False, extra_info
        self._update_grasped_object_pose()

        object_pos = self.data.xpos[self._object_body_id].copy()
        object_xy_error = float(np.linalg.norm(object_pos[:2] - self.current_task.target_pos[:2]))
        ee_error = float(np.linalg.norm(target_ee_pos - self.data.site_xpos[self.names.ee_site_id]))
        phase_success = self.object_grasped and object_xy_error <= 0.055 and object_pos[2] >= 0.075
        extra_info["ee_error"] = ee_error
        extra_info["object_xy_error"] = object_xy_error
        extra_info["object_z"] = float(object_pos[2])
        if phase_success:
            self.phase_manager.set_phase(Phase.PLACE)
        return "AT_PLACE" if phase_success else "PLACE_APPROACH_MISS", sim_steps, True, phase_success, extra_info

    def _execute_place(
        self,
        decoded: DecodedAction,
    ) -> tuple[str, int, bool, bool, dict[str, float | int | str | bool]]:
        del decoded
        if self.current_task is None:
            return "NO_TASK", 0, True, False, {"failure_reason": "missing_task"}
        if not self.object_grasped:
            return "NO_GRASP", 0, True, False, {"failure_reason": "object_not_grasped"}

        q_hold = self.data.qpos[self.names.arm_qposadr].copy()
        self._run_pd_hold(q_hold, self._gripper_open_q(), 120)
        placed_pos = self._placed_object_pos()
        self.object_grasped = False
        set_freejoint_pose(self.data, self.names, placed_pos, self.grasp_object_quat,
                           color=self._pick_color)
        self.data.qvel[self.names.block_dofadr[self._pick_color]:self.names.block_dofadr[self._pick_color] + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self._run_pd_hold(q_hold, self._gripper_open_q(), 80)

        object_pos = self.data.xpos[self._object_body_id].copy()
        object_speed = float(np.linalg.norm(self.data.cvel[self._object_body_id, :3]))
        object_xy_error = float(np.linalg.norm(object_pos[:2] - self.current_task.target_pos[:2]))
        object_in_target = self._object_in_target()
        finger_q = float(self.data.qpos[self.names.finger_r_qposadr])
        phase_success = object_in_target and object_speed <= 0.05
        extra_info: dict[str, float | int | str | bool] = {
            "object_xy_error": object_xy_error,
            "object_z": float(object_pos[2]),
            "object_speed": object_speed,
            "object_in_target": object_in_target,
            "finger_q": finger_q,
            "finger_open_q": self._gripper_open_q(),
            "finger_closed_q": self._gripper_closed_q(),
        }
        if phase_success:
            self.phase_manager.set_phase(Phase.RETREAT)
        return "PLACED" if phase_success else "PLACE_FAIL", 200, True, phase_success, extra_info

    def _execute_home(
        self,
        decoded: DecodedAction,
    ) -> tuple[str, int, bool, bool, dict[str, float | int | str | bool]]:
        del decoded
        q_start = self.data.qpos[self.names.arm_qposadr].copy()
        trajectory = make_joint_space_trajectory(
            q_start,
            self.home_q,
            control_hz=1.0 / float(self.model.opt.timestep),
            q_speed=0.75,
        )
        sim_steps = self.trajectory_executor.execute(
            trajectory,
            gripper_target=self._gripper_open_q(),
            settle_steps=120,
            post_step=self._after_internal_step,
        )
        q_error = float(np.linalg.norm(self.data.qpos[self.names.arm_qposadr] - self.home_q))
        object_in_target = self._object_in_target()
        phase_success = q_error <= 0.15 and object_in_target and not self.object_grasped
        extra_info: dict[str, float | int | str | bool] = {
            "q_error": q_error,
            "object_in_target": object_in_target,
        }
        if phase_success:
            self.phase_manager.set_phase(Phase.DONE)
        return "DONE" if phase_success else "HOME_MISS", sim_steps, True, phase_success, extra_info

    def _execute_recovery(
        self,
        decoded: DecodedAction,
    ) -> tuple[str, int, bool, bool, dict[str, float | int | str | bool]]:
        del decoded
        current_phase = self.phase_manager.phase
        target_pos = self.data.site_xpos[self.names.ee_site_id].copy()
        target_pos[0] = np.clip(
            target_pos[0],
            self.feasibility.workspace.x[0] + 0.02,
            self.feasibility.workspace.x[1] - 0.02,
        )
        target_pos[1] = np.clip(
            target_pos[1],
            self.feasibility.workspace.y[0] + 0.02,
            self.feasibility.workspace.y[1] - 0.02,
        )
        target_pos[2] = min(target_pos[2] + 0.08, self.feasibility.workspace.z[1] - 0.02)
        feasibility = self.feasibility.check_workspace(target_pos)
        if not feasibility.feasible:
            return "WORKSPACE_FAIL", 0, False, False, {
                "target_x": float(target_pos[0]),
                "target_y": float(target_pos[1]),
                "target_z": float(target_pos[2]),
                "failure_reason": feasibility.reason,
                "recovery_from_phase": current_phase.name,
            }

        gripper_target = self._gripper_closed_q() if self.object_grasped else self._gripper_open_q()
        status, sim_steps, ik_success, extra_info = self._move_ee_to_target(
            target_pos,
            gripper_target=gripper_target,
            q_speed=0.6,
            settle_steps=80,
            target_yaw=self._current_ee_yaw(),
            post_step=self._update_grasped_object_pose if self.object_grasped else None,
        )
        extra_info["recovery_from_phase"] = current_phase.name
        if not ik_success:
            return status, sim_steps, False, False, extra_info

        if self.object_grasped:
            self._update_grasped_object_pose()
            self.phase_manager.set_phase(Phase.LIFT)
        else:
            self.phase_manager.set_phase(Phase.OBSERVE_OBJECT)
        return "RECOVERED", sim_steps, True, False, extra_info

    def _move_ee_to_target(
        self,
        target_pos: np.ndarray,
        gripper_target: float,
        q_speed: float,
        settle_steps: int,
        target_yaw: float = 0.0,
        post_step=None,
    ) -> tuple[str, int, bool, dict[str, float | int | str | bool]]:
        q_start = self.data.qpos[self.names.arm_qposadr].copy()
        ik_result = self.ik.solve_top_down_yaw_free(q_start, target_pos, target_yaw=target_yaw)
        extra_info: dict[str, float | int | str | bool] = {
            "target_x": float(target_pos[0]),
            "target_y": float(target_pos[1]),
            "target_z": float(target_pos[2]),
            "target_yaw": float(target_yaw),
            "ik_residual": float(ik_result.residual_norm),
            "ik_iterations": int(ik_result.iterations),
        }
        if not ik_result.success:
            extra_info["failure_reason"] = ik_result.reason
            return "IK_FAIL", 0, False, extra_info

        trajectory = make_joint_space_trajectory(
            q_start,
            ik_result.q_goal,
            control_hz=1.0 / float(self.model.opt.timestep),
            q_speed=q_speed,
        )
        sim_steps = self.trajectory_executor.execute(
            trajectory,
            gripper_target=gripper_target,
            settle_steps=settle_steps,
            post_step=self._combine_post_step(post_step),
        )
        return "SETTLED", sim_steps, True, extra_info

    def _current_ee_yaw(self) -> float:
        rotation = self.data.site_xmat[self.names.ee_site_id].reshape(3, 3)
        return float(np.arctan2(rotation[1, 0], rotation[0, 0]))

    def _execute_skeleton_command(self, decoded: DecodedAction) -> tuple[str, int]:
        if decoded.command == Command.HOME:
            target_arm_q = self.home_q
            gripper_target = self._gripper_open_q()
        elif decoded.command == Command.GRASP or decoded.gripper_close:
            target_arm_q = self.data.qpos[self.names.arm_qposadr].copy()
            gripper_target = self._gripper_closed_q()
        elif decoded.command == Command.PLACE:
            target_arm_q = self.data.qpos[self.names.arm_qposadr].copy()
            gripper_target = self._gripper_open_q()
        else:
            target_arm_q = self.data.qpos[self.names.arm_qposadr].copy()
            gripper_target = float(self.data.qpos[self.names.controlled_qposadr[-1]])

        self._run_pd_hold(target_arm_q, gripper_target, self.frame_skip)
        return "SETTLED", self.frame_skip

    def _run_pd_hold(self, target_arm_q: np.ndarray, target_gripper_q: float, steps: int) -> None:
        kp_arm = 80.0
        kd_arm = 4.0
        kp_gripper = 80.0
        kd_gripper = 4.0
        ctrl = np.zeros(self.model.nu, dtype=np.float64)

        for _ in range(steps):
            arm_q = self.data.qpos[self.names.arm_qposadr]
            arm_qd = self.data.qvel[self.names.arm_dofadr]
            arm_tau = kp_arm * (target_arm_q - arm_q) - kd_arm * arm_qd
            arm_tau = arm_tau + self.data.qfrc_bias[self.names.arm_dofadr]
            arm_limits = self.model.actuator_ctrlrange[self.names.arm_actuator_ids]
            arm_tau = np.clip(arm_tau, arm_limits[:, 0], arm_limits[:, 1])

            gripper_q = self.data.qpos[self.names.controlled_qposadr[-1]]
            gripper_qd = self.data.qvel[self.names.controlled_dofadr[-1]]
            gripper_tau = kp_gripper * (target_gripper_q - gripper_q) - kd_gripper * gripper_qd
            gripper_limit = self.model.actuator_ctrlrange[self.names.gripper_actuator_id]
            gripper_tau = float(np.clip(gripper_tau, gripper_limit[0], gripper_limit[1]))

            ctrl[:] = 0.0
            ctrl[self.names.arm_actuator_ids] = arm_tau
            ctrl[self.names.gripper_actuator_id] = gripper_tau
            self.data.ctrl[:] = ctrl
            mujoco.mj_step(self.model, self.data)
            self._after_internal_step()

    def _combine_post_step(self, post_step):
        if post_step is None:
            return self._after_internal_step

        def _combined() -> None:
            post_step()
            self._after_internal_step()

        return _combined

    def _after_internal_step(self) -> None:
        if self.post_mj_step_callback is not None:
            self.post_mj_step_callback()

    def _apply_home_pose(self) -> None:
        self.data.qpos[self.names.arm_qposadr] = self.home_q
        gripper_open = self._gripper_open_q()
        self.data.qpos[self.names.finger_r_qposadr] = gripper_open
        self.data.qpos[self.names.finger_l_qposadr] = gripper_open
        self.data.qvel[self.names.controlled_dofadr] = 0.0
        self.data.qvel[self.names.finger_l_dofadr] = 0.0

    def _apply_task_sample(self, sample: TaskSample) -> None:
        _ID_QUAT = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        # 집을 블록
        set_freejoint_pose(self.data, self.names, sample.object_pos, sample.object_quat,
                           color=sample.pick_color)
        # 방관자 블록
        for color, pos in sample.bystander_poses.items():
            set_freejoint_pose(self.data, self.names, pos, _ID_QUAT, color=color)
        # stack: 타겟 블록 (center z = BLOCK_Z, target_pos[2] = STACK_Z)
        if sample.task_type == "stack" and sample.target_color is not None:
            tgt_center = np.array([sample.target_pos[0], sample.target_pos[1], 0.023],
                                   dtype=np.float64)
            set_freejoint_pose(self.data, self.names, tgt_center, _ID_QUAT,
                               color=sample.target_color)
        body_mass = self.model.body_mass[self._object_body_id]
        if body_mass > 0.0:
            self.model.body_mass[self._object_body_id] = sample.object_mass

    def _update_grasped_object_pose(self) -> None:
        if not self.object_grasped:
            return
        ee_pos = self.data.site_xpos[self.names.ee_site_id].copy()
        object_pos = ee_pos + self.grasp_offset_pos
        set_freejoint_pose(self.data, self.names, object_pos, self.grasp_object_quat,
                           color=self._pick_color)
        mujoco.mj_forward(self.model, self.data)

    def _placed_object_pos(self) -> np.ndarray:
        if self.current_task is None:
            return self.data.xpos[self._object_body_id].copy()
        pos = self.current_task.target_pos.copy()
        if self.current_task.task_type == "stack":
            pos[2] = self.current_task.target_pos[2]  # 0.063 = target center
        else:
            pos[2] = 0.023
        return pos

    def _object_in_target(self) -> bool:
        if self.current_task is None:
            return False
        object_pos = self.data.xpos[self._object_body_id]
        xy_error = float(np.linalg.norm(object_pos[:2] - self.current_task.target_pos[:2]))
        if self.current_task.task_type == "stack":
            return bool(xy_error <= 0.03 and 0.048 <= object_pos[2] <= 0.078)
        return bool(xy_error <= 0.06 and 0.0 <= object_pos[2] <= 0.08)

    def _gripper_open_q(self) -> float:
        return float(self.names.joint_ranges[-1, 0])

    def _gripper_closed_q(self) -> float:
        return float(self.names.joint_ranges[-1, 1])

    def _gripper_grasp_min_q(self) -> float:
        open_q = self._gripper_open_q()
        closed_q = self._gripper_closed_q()
        return float(open_q + 0.45 * (closed_q - open_q))

    def _observe(self) -> dict[str, np.ndarray]:
        if self.current_task is None:
            raise RuntimeError("Environment must be reset before observation")
        state = SnapshotState(
            phase_id=int(self.phase_manager.phase),
            time_in_phase=self.phase_manager.time_in_phase,
            attempt_count=self.phase_manager.attempt_count,
            prev_command_id=self.prev_command_id,
            prev_result_id=int(self.prev_result),
            prev_reward=self.prev_reward,
            object_grasped=self.object_grasped,
            contact_probability=1.0 if self.object_grasped else 0.0,
        )
        slot_state = self._build_slot_state()
        slot_diff_emb = self._cached_slot_diff_emb
        obs = self.observer.observe(slot_state, state, slot_diff_emb=slot_diff_emb)
        if self._wm_model is not None and self._wm_h is not None:
            obs["rssm_latent"] = self._wm_step(obs)
        _COLORS  = ["red", "green", "blue"]
        _TARGETS = ["red", "green", "blue", "basket"]
        task_oh = [1.0, 0.0] if self.current_task.task_type == "pick_place" else [0.0, 1.0]
        obj_oh  = [float(self._pick_color == c) for c in _COLORS]
        tgt_lbl = self.current_task.target_color if self.current_task.target_color else "basket"
        tgt_oh  = [float(tgt_lbl == t) for t in _TARGETS]
        obs["cmd"] = np.array(task_oh + obj_oh + tgt_oh, dtype=np.float32)
        return obs

    def _wm_step(self, obs: dict) -> np.ndarray:
        """GRU hidden state 한 step 업데이트 후 64-dim rssm_latent 반환."""
        import torch
        phase_id = int(self.phase_manager.phase)
        task = obs["task"]
        goal_xy = task[:2] if phase_id <= int(Phase.LIFT) else task[2:4]
        phase_dest = encode_phase_destination_2d(phase_id=phase_id, goal_xy_world=goal_xy)
        x_t = np.concatenate([obs["slot_diff"], obs["robot"], phase_dest]).astype(np.float32)
        x_t_tensor = torch.tensor(x_t).unsqueeze(0)
        with torch.no_grad():
            self._wm_h, _, _, _ = self._wm_model(x_t_tensor, self._wm_h)
            latent = self._wm_model.rssm_latent(self._wm_h).squeeze(0).numpy()
        return latent

    def _estimate_pose(self):
        if self.current_task is None:
            raise RuntimeError("Environment must be reset before pose estimation")
        if self.pose_source == "slot":
            slot_state = self._build_slot_state()
            object_pos = np.array(
                [
                    float(slot_state.object_xy[0]),
                    float(slot_state.object_xy[1]),
                    float(self.current_task.object_pos[2]),
                ],
                dtype=np.float64,
            )
            target_pos = self.current_task.target_pos.copy()
            target_pos[:2] = slot_state.target_xy.astype(np.float64)
            self.last_pose_estimate = PoseEstimate(
                object_pos=object_pos,
                object_quat=_yaw_to_quat_wxyz(slot_state.object_yaw),
                target_pos=target_pos,
                target_yaw=float(slot_state.target_yaw),
                object_confidence=float(slot_state.object_confidence),
                target_confidence=float(slot_state.target_confidence),
                source="slot",
            )
            return self.last_pose_estimate
        self.last_pose_estimate = self.pose_provider.estimate(self.current_task, self.rng)
        return self.last_pose_estimate

    def _build_slot_state(self) -> SlotState:
        """'slot' 모드: SlotEmbedder를 interval마다 실행 후 SlotStateBridge. 'zeros': GT XY."""
        if self.image_embedding_mode == "slot" and self.slot_embedder is not None:
            should_run = (
                self._cached_curr_slots is None
                or self.step_count % self.image_embedding_interval == 0
            )
            if should_run:
                if self._embed_injected:
                    # 배치 추론 결과가 이미 주입됨 — 캐시 그대로 사용
                    self._embed_injected = False
                elif self._slot_embed_deferred and self._cached_curr_slots is not None:
                    # deferred 모드: 렌더만 하고 NN 추론은 나중에 배치로 수행
                    self._slot_embed_pending_img = self.slot_embedder.render_and_preprocess(
                        self.model, self.data
                    )
                    # 이번 step obs는 이전 캐시 사용 (inject 후 _observe() 재호출로 갱신됨)
                else:
                    # 일반 경로 (초기화 시 또는 deferred 비활성)
                    emb, curr_slots = self.slot_embedder.embed(self.model, self.data)
                    self._cached_slot_diff_emb = emb
                    self._cached_curr_slots = curr_slots
                    # 에피소드 첫 관측 시 GT proximity로 grounding 초기화
                    if not self.slot_state_bridge_grounded:
                        self._init_grounding_from_gt(curr_slots)
                        self.slot_state_bridge_grounded = True
            return self.slot_state_bridge.estimate(self._cached_curr_slots)
        # zeros 모드: GT 위치 직접 사용
        pose = self.pose_provider.estimate(self.current_task, self.rng)
        self.last_pose_estimate = pose
        return SlotState(
            object_xy=pose.object_pos[:2].astype(np.float32),
            target_xy=pose.target_pos[:2].astype(np.float32),
        )

    def inject_slot_result(self, emb: np.ndarray, curr_slots: dict) -> None:
        """배치 추론 결과를 주입. 이후 _observe() 호출 시 신선한 임베딩을 사용한다."""
        self._cached_slot_diff_emb = emb
        self._cached_curr_slots = curr_slots
        if not self.slot_state_bridge_grounded and self.slot_state_bridge is not None:
            self._init_grounding_from_gt(curr_slots)
            self.slot_state_bridge_grounded = True
        if self.slot_embedder is not None:
            self.slot_embedder._prev_slots = curr_slots
        self._slot_embed_pending_img = None
        self._embed_injected = True

    def _init_grounding_from_gt(self, curr_slots: dict) -> None:
        """GT world XY와 slot XY를 비교해 closest slot을 grounding으로 설정."""
        pose = self.pose_provider.estimate(self.current_task, self.rng)
        self.last_pose_estimate = pose
        obj_world = pose.object_pos[:2]
        tgt_world = pose.target_pos[:2]
        slot_worlds = np.array([
            self.slot_state_bridge._norm_to_world(xy)
            for xy in curr_slots["xy"]
        ])  # (N, 2)
        presents = curr_slots["present"][:, 0]  # (N,)
        weights = np.where(presents > 0.5, 1.0, 10.0)
        obj_idx = int(np.argmin(
            np.linalg.norm(slot_worlds - obj_world, axis=1) * weights
        ))
        tgt_idx = int(np.argmin(
            np.linalg.norm(slot_worlds - tgt_world, axis=1) * weights
        ))
        self.slot_state_bridge.set_grounding(obj_idx, tgt_idx)

    def _info(
        self,
        command: Command | None,
        raw_command: Command | None,
        command_was_masked: bool,
        valid_command: bool,
        executor_status: str,
        ik_success: bool,
        sim_steps: int,
        reward_components: dict[str, float],
        phase_success: bool,
        phase_failure: bool,
        extra_info: dict | None = None,
        phase_before: Phase | None = None,
    ) -> dict:
        if self.current_task is None:
            object_in_target = False
        else:
            object_in_target = self._object_in_target()

        info = {
            "phase_before": self.phase_manager.phase.name if phase_before is None else phase_before.name,
            "phase": self.phase_manager.phase.name,
            "phase_after": self.phase_manager.phase.name,
            "command": None if command is None else command.name,
            "raw_command": None if raw_command is None else raw_command.name,
            "command_was_masked": bool(command_was_masked),
            "valid_command": bool(valid_command),
            "phase_success": bool(phase_success),
            "phase_failure": bool(phase_failure),
            "executor_status": executor_status,
            "ik_success": bool(ik_success),
            "object_grasped": bool(self.object_grasped),
            "object_in_target": object_in_target,
            "dropped": bool(self.dropped),
            "image_embedding_mode": self.image_embedding_mode,
            "pose_source": self.pose_source,
            "pose_object_confidence": 0.0 if self.last_pose_estimate is None else self.last_pose_estimate.object_confidence,
            "pose_target_confidence": 0.0 if self.last_pose_estimate is None else self.last_pose_estimate.target_confidence,
            "reward_components": reward_components,
            "sim_steps": int(sim_steps),
            "attempt_count": int(
                extra_info.get("attempt_count", self.phase_manager.attempt_count)
                if extra_info
                else self.phase_manager.attempt_count
            ),
            "max_phase_failures": int(self.max_phase_failures),
        }
        planner_fail_reason = self._planner_fail_reason(
            valid_command=valid_command,
            executor_status=executor_status,
            timeout=bool(extra_info.get("timeout", False)) if extra_info else False,
            extra_info=extra_info,
        )
        info["planner_fail_reason"] = planner_fail_reason
        info["planner_fail_class"] = self._planner_fail_class(
            planner_fail_reason,
            executor_status,
        )
        if extra_info:
            info.update(extra_info)
        return info

    def _planner_fail_reason(
        self,
        valid_command: bool,
        executor_status: str,
        timeout: bool,
        extra_info: dict | None,
    ) -> str:
        if not valid_command:
            return "INVALID_COMMAND"
        if timeout:
            return "TIMEOUT"
        if executor_status not in FAILURE_STATUSES:
            return ""
        if extra_info and extra_info.get("failure_reason"):
            return str(extra_info["failure_reason"])
        return executor_status

    def _planner_fail_class(self, reason: str, executor_status: str) -> str:
        if not reason:
            return ""
        normalized = f"{reason} {executor_status}".upper()
        if "INVALID" in normalized:
            return "COMMAND"
        if "TIMEOUT" in normalized:
            return "TIMEOUT"
        if "WORKSPACE" in normalized:
            return "WORKSPACE"
        if "COLLISION" in normalized or "FLOOR" in normalized:
            return "COLLISION"
        if (
            "IK" in normalized
            or "JOINT_LIMIT" in normalized
            or "MANIPULABILITY" in normalized
            or "UNREACHABLE" in normalized
            or "RESIDUAL" in normalized
            or "GEOMETRIC" in normalized
        ):
            return "IK_OR_PLANNER"
        if "GRASP" in normalized:
            return "GRASP"
        if "DROP" in normalized:
            return "DROP"
        return "EXECUTION"


def _quat_wxyz_to_yaw(quat: np.ndarray) -> float:
    w, x, y, z = [float(value) for value in np.asarray(quat, dtype=np.float64)]
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def _yaw_to_quat_wxyz(yaw: float) -> np.ndarray:
    half = 0.5 * float(yaw)
    return np.array([np.cos(half), 0.0, 0.0, np.sin(half)], dtype=np.float64)
