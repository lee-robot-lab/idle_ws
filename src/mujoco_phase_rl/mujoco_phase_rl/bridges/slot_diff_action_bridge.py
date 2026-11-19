# ================================================================
# bridges/slot_diff_action_bridge.py
# 설명: RealActionBridgeNode를 상속해 SlotDiff 방식 obs를 주입하는 실기체 브리지.
#       _build_policy_obs()를 override해 101/165-dim PPO obs를 우리 포맷으로 대체함.
#       카메라 프레임마다 SlotDiffRealProvider를 업데이트하나, phase 내부에서
#       OBSERVE_OBJECT 이외의 phase는 캐시를 재사용한다.
# 사용법: from mujoco_phase_rl.bridges.slot_diff_action_bridge import SlotDiffActionBridge
# ================================================================
from __future__ import annotations

import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from mujoco_phase_rl.bridges.real_action_bridge import RealActionBridgeNode
from mujoco_phase_rl.bridges.real_obs_builder import RealObsBuilder
from mujoco_phase_rl.bridges.slot_diff_real_provider import SlotDiffRealProvider
from mujoco_phase_rl.bridges.real_phase_diagnostics import FusedState
from mujoco_phase_rl.perception.snapshot_observer import SnapshotState


class SlotDiffActionBridge(RealActionBridgeNode):
    """SlotDiff 기반 시각 관측을 PPO obs로 변환하는 실기체 액션 브리지.

    팀의 DirectVisionActionBridgeNode와 동일한 구조지만, Stage1ColorNetProvider
    대신 SlotDiffRealProvider를 사용하고 우리 고유의 165-dim obs 포맷을 생성한다.
    """

    def __init__(
        self,
        rclpy_module: Any,
        action_config: Any,
        stage1_ckpt: str | Path,
        slot_diff_ckpt: str | Path,
        color_net_ckpt: str | Path,
        pick_color: str,
        target_color: str,
        camera_device: str = "0",
        camera_width: int = 1280,
        camera_height: int = 720,
        camera_buffer_size: int = 1,
        camera_flush_frames: int = 0,
        camera_rate_hz: float = 10.0,
        device: str = "cpu",
    ) -> None:
        super().__init__(rclpy_module, action_config)
        self._slot_provider = SlotDiffRealProvider(
            stage1_ckpt=stage1_ckpt,
            slot_diff_ckpt=slot_diff_ckpt,
            color_net_ckpt=color_net_ckpt,
            pick_color=pick_color,
            target_color=target_color,
            device=device,
        )
        self._obs_builder = RealObsBuilder()
        self._camera_flush_frames = int(camera_flush_frames)
        self._cap = self._open_camera(camera_device, camera_width, camera_height, camera_buffer_size)
        self.node.create_timer(
            1.0 / max(float(camera_rate_hz), 1.0e-6),
            self._on_camera_timer,
        )

    @staticmethod
    def _open_camera(device_arg: str, width: int, height: int, buffer_size: int):
        device = int(device_arg) if str(device_arg).isdigit() else str(device_arg)
        cap = cv2.VideoCapture(device)
        if not cap.isOpened():
            raise RuntimeError(f"failed to open camera device: {device_arg}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))
        cap.set(cv2.CAP_PROP_BUFFERSIZE, float(max(1, buffer_size)))
        return cap

    def _on_camera_timer(self) -> None:
        for _ in range(self._camera_flush_frames):
            self._cap.grab()
        ok, frame_bgr = self._cap.read()
        if not ok or frame_bgr is None:
            return
        fused = getattr(self, "latest_fused", None)
        phase = fused.phase if fused is not None else None
        if phase is not None:
            self._slot_provider.update(frame_bgr, phase)

    def close(self) -> None:
        if getattr(self, "_cap", None) is not None:
            self._cap.release()
        if getattr(self, "_slot_provider", None) is not None:
            self._slot_provider.close()
        super().close()

    def _build_policy_obs(self, fused: FusedState) -> dict[str, np.ndarray]:
        q = fused.q if fused.q is not None else np.zeros(7, dtype=np.float32)
        ee_pos = fused.ee_pos if fused.ee_pos is not None else np.zeros(3, dtype=np.float32)
        robot_vec = np.concatenate([
            q[:6].astype(np.float32),
            ee_pos.astype(np.float32),
            np.array([float(fused.gripper_opening), float(fused.object_grasped)], dtype=np.float32),
        ])

        snap = SnapshotState(
            phase_id=int(fused.phase),
            time_in_phase=float(time.monotonic() - self.phase_started_s),
            attempt_count=int(self.attempt_count),
            prev_command_id=int(self.prev_command) if self.prev_command is not None else None,
            prev_result_id=int(self.prev_result),
            prev_reward=float(self.prev_reward),
            object_grasped=bool(fused.object_grasped),
            contact_probability=0.0,
        )

        return self._obs_builder.build(
            robot_vec=robot_vec,
            slot_state=self._slot_provider.slot_state,
            state=snap,
            slot_diff_emb=self._slot_provider.slot_diff_emb,
        )
