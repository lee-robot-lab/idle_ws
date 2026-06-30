# ================================================================
# test_real_bc_recorder
# 설명: RealBCRecorder 단위 테스트 — JSONL 기록 및 episode 구조 검증
# ================================================================
import json
import tempfile
from pathlib import Path
import numpy as np
from mujoco_phase_rl.bridges.real_bc_recorder import RealBCRecorder


def _sample_obs() -> np.ndarray:
    return np.zeros(165, dtype=np.float32)


def _sample_action() -> np.ndarray:
    return np.ones(14, dtype=np.float32)


def test_record_step_writes_jsonl():
    with tempfile.TemporaryDirectory() as tmp:
        recorder = RealBCRecorder(output_dir=Path(tmp))
        recorder.record_step(obs=_sample_obs(), action=_sample_action(), phase="OBSERVE_OBJECT")
        recorder.record_episode_end(success=True)

        ep_dir = Path(tmp) / "episode_0"
        steps_path = ep_dir / "steps.jsonl"
        assert steps_path.exists(), "steps.jsonl not created"
        rows = [json.loads(l) for l in steps_path.read_text().strip().splitlines()]
        assert len(rows) == 1
        assert rows[0]["phase"] == "OBSERVE_OBJECT"
        assert len(rows[0]["obs"]) == 165
        assert len(rows[0]["action"]) == 14


def test_episode_meta_written_on_end():
    with tempfile.TemporaryDirectory() as tmp:
        recorder = RealBCRecorder(output_dir=Path(tmp))
        recorder.record_step(obs=_sample_obs(), action=_sample_action(), phase="GRASP")
        recorder.record_episode_end(success=False)

        meta_path = Path(tmp) / "episode_0" / "meta.json"
        assert meta_path.exists()
        meta = json.loads(meta_path.read_text())
        assert meta["success"] is False
        assert meta["n_steps"] == 1


def test_multiple_episodes_get_separate_dirs():
    with tempfile.TemporaryDirectory() as tmp:
        recorder = RealBCRecorder(output_dir=Path(tmp))
        for _ in range(3):
            recorder.record_step(obs=_sample_obs(), action=_sample_action(), phase="LIFT")
            recorder.record_episode_end(success=True)

        for i in range(3):
            assert (Path(tmp) / f"episode_{i}").is_dir()


def test_only_successful_episodes_filter():
    with tempfile.TemporaryDirectory() as tmp:
        recorder = RealBCRecorder(output_dir=Path(tmp))
        recorder.record_step(obs=_sample_obs(), action=_sample_action(), phase="GRASP")
        recorder.record_episode_end(success=False)
        recorder.record_step(obs=_sample_obs(), action=_sample_action(), phase="GRASP")
        recorder.record_episode_end(success=True)

        success_eps = recorder.list_successful_episodes()
        assert len(success_eps) == 1
        assert "episode_1" in str(success_eps[0])
