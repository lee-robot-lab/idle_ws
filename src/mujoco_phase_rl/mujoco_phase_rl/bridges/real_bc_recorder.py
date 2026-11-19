# ================================================================
# bridges/real_bc_recorder.py
# 설명: 실기체 에피소드에서 BC 파인튜닝용 (obs, action) 쌍을 JSONL로 기록.
#       성공 에피소드만 학습에 사용하므로 meta.json에 success 플래그 기록.
# 사용법: from mujoco_phase_rl.bridges.real_bc_recorder import RealBCRecorder
# ================================================================
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

import numpy as np


class RealBCRecorder:
    """BC 파인튜닝용 (obs, action) 에피소드 기록기.

    에피소드별 디렉토리(episode_N/)에 steps.jsonl과 meta.json을 저장한다.
    성공 에피소드 경로 목록은 list_successful_episodes()로 조회한다.
    """

    def __init__(self, output_dir: str | Path) -> None:
        self._output_dir = Path(output_dir)
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self._episode_idx = 0
        self._step_idx = 0
        self._steps_file = None
        self._ep_dir: Path | None = None
        self._ep_started_s: float | None = None

    def record_step(
        self,
        obs: np.ndarray,
        action: np.ndarray,
        phase: str,
        step_idx: int | None = None,
    ) -> None:
        if self._steps_file is None:
            self._start_episode()

        row: dict[str, Any] = {
            "step": step_idx if step_idx is not None else self._step_idx,
            "phase": phase,
            "obs": obs.tolist(),
            "action": action.tolist(),
            "stamp_ns": time.time_ns(),
        }
        self._steps_file.write(json.dumps(row, sort_keys=True) + "\n")
        self._step_idx += 1

    def record_episode_end(self, success: bool) -> None:
        if self._steps_file is not None:
            self._steps_file.flush()
            self._steps_file.close()
            self._steps_file = None

        if self._ep_dir is not None:
            elapsed = (
                float(time.monotonic() - self._ep_started_s)
                if self._ep_started_s is not None
                else None
            )
            meta = {
                "episode": self._episode_idx,
                "success": bool(success),
                "n_steps": int(self._step_idx),
                "elapsed_s": elapsed,
                "stamp_ns": time.time_ns(),
            }
            (self._ep_dir / "meta.json").write_text(json.dumps(meta, indent=2))

        self._episode_idx += 1
        self._step_idx = 0
        self._ep_dir = None
        self._ep_started_s = None

    def list_successful_episodes(self) -> list[Path]:
        result = []
        for meta_path in sorted(self._output_dir.glob("episode_*/meta.json")):
            try:
                meta = json.loads(meta_path.read_text())
                if meta.get("success"):
                    result.append(meta_path.parent)
            except (json.JSONDecodeError, OSError):
                continue
        return result

    def _start_episode(self) -> None:
        self._ep_dir = self._output_dir / f"episode_{self._episode_idx}"
        self._ep_dir.mkdir(parents=True, exist_ok=True)
        self._steps_file = (self._ep_dir / "steps.jsonl").open("w", encoding="utf-8")
        self._step_idx = 0
        self._ep_started_s = time.monotonic()

    def close(self) -> None:
        if self._steps_file is not None:
            self._steps_file.flush()
            self._steps_file.close()
            self._steps_file = None
