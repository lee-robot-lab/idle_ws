# ROS2 Real-Data Pipeline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 실기체에서 PPO 데모를 실행하고 실제 이미지 기반 val_pool을 구성해 sim fine-tuning을 실데이터 분포에 맞게 적응시키는 파이프라인을 완성한다.

**Architecture:**
세 단계로 분리한다.
1. **ppo_supervisor_node**: ROS2 노드 — 카메라·관절 구독 → SlotEmbedder 추론 → PPO shadow/execute → FSM 커맨드 publish → 에피소드 녹화
2. **build_real_val_pool**: 녹화된 에피소드(이미지+task)를 finetune_stack_robust.py가 쓰는 val_pool 포맷으로 변환
3. **finetune_from_real**: 실이미지 기반 val_pool로 PPO를 domain-adaptive fine-tuning

perception은 SlotEmbedder를 그대로 재사용한다. 새 ROS2 gym Env 레이어는 만들지 않는다(공수 대비 효과 낮음). 실기체 안전을 위해 첫 배포는 shadow mode로 시작한다.

**Tech Stack:** Python 3.10, ROS2 Humble (rclpy), stable-baselines3, PyTorch, cv_bridge, sensor_msgs, msgs (커스텀)

## Global Constraints

- `source /opt/ros/humble/setup.bash && source ~/idle_ws/install/setup.bash` 후 실행
- 실기체 코드 수정 시 시뮬레이션 먼저 검증
- ppo_supervisor_node는 `/pickplace/command` 를 기존 `task_fsm_node`에 publish
- 기존 `task_fsm_node`, `plan_compute_node`, `can_bridge_node`는 수정하지 않는다
- shadow mode가 기본값 (`--execute` 플래그로만 실행 활성화)
- 에피소드 녹화 포맷: `.npz` per episode, manifest `episodes.json`

---

### Task 1: PPO Supervisor Node

PPO를 실기체에 올리는 ROS2 노드. 초기에는 shadow mode (action 계산만, FSM 미연결) 로 동작하고, `--execute` 플래그로 실행 모드 전환.

**Files:**
- Create: `mujoco_phase_rl/mujoco_phase_rl/ros2/__init__.py`
- Create: `mujoco_phase_rl/mujoco_phase_rl/ros2/ppo_supervisor_node.py`
- Modify: `mujoco_phase_rl/setup.py` (entry_points 추가)

**Interfaces:**
- Subscribes: `/camera/camera/color/image_raw` (`sensor_msgs/Image`)
- Subscribes: `/joint_states` (`sensor_msgs/JointState`)
- Subscribes: `/task_fsm/status` (`std_msgs/String`)
- Publishes: `/pickplace/command` (`msgs/PickPlaceCommand`) — execute mode만
- Publishes: `/ppo_supervisor/status` (`std_msgs/String`) — JSON 로그
- Produces: episode `.npz` files (`obs`, `action`, `reward`, `done`)

- [ ] **Step 1: 디렉토리 생성 및 `__init__.py`**

```bash
mkdir -p mujoco_phase_rl/mujoco_phase_rl/ros2
touch mujoco_phase_rl/mujoco_phase_rl/ros2/__init__.py
```

- [ ] **Step 2: `ppo_supervisor_node.py` 작성**

```python
# ================================================================
# ppo_supervisor_node.py
# 설명: PPO 정책을 실기체에 연결하는 ROS2 supervisor.
#       shadow mode(기본)에서는 action 계산+기록만, execute mode에서
#       /pickplace/command를 publish해 기존 FSM을 구동한다.
# 사용법:
#   ros2 run mujoco_phase_rl ppo_supervisor \
#     --ros-args -p model_path:=outputs/ppo_stack_pg_s0/final_model.zip \
#                -p execute:=false -p record_dir:=/tmp/episodes
# ================================================================
from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String

_WS_ROOT = Path(__file__).resolve().parents[5]
_CKPT_ROOT = _WS_ROOT / "checkpoints"
_DEFAULT_STAGE1    = str(_CKPT_ROOT / "stage1_v2"    / "best.pt")
_DEFAULT_SLOT_DIFF = str(_CKPT_ROOT / "slot_diff"    / "best.pt")
_DEFAULT_COLOR_NET = str(_CKPT_ROOT / "color_net_v2" / "best.pt")

_BLOCK_COLORS = ("red", "green", "blue")
_COLOR_OH = {c: i for i, c in enumerate(_BLOCK_COLORS)}
_TARGETS   = ("red", "green", "blue", "basket")
_TARGET_OH = {t: i for i, t in enumerate(_TARGETS)}


class PPOSupervisorNode(Node):
    """카메라+관절 상태 → SlotEmbedder → PPO → PickPlaceCommand."""

    def __init__(self) -> None:
        super().__init__("ppo_supervisor_node")
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_img: np.ndarray | None = None
        self._latest_joints: np.ndarray | None = None
        self._fsm_status: str = "IDLE"

        # ROS params
        self.declare_parameter("model_path", "outputs/ppo_stack_pg_s0/final_model.zip")
        self.declare_parameter("stage1_ckpt",    _DEFAULT_STAGE1)
        self.declare_parameter("slot_diff_ckpt", _DEFAULT_SLOT_DIFF)
        self.declare_parameter("color_net_ckpt", _DEFAULT_COLOR_NET)
        self.declare_parameter("slot_device",    "cuda")
        self.declare_parameter("execute",        False)
        self.declare_parameter("record_dir",     "")
        self.declare_parameter("pick_color",     "red")
        self.declare_parameter("task_type",      "pick_place")
        self.declare_parameter("target_color",   "basket")

        model_path    = str(self.get_parameter("model_path").value)
        stage1_ckpt   = str(self.get_parameter("stage1_ckpt").value)
        slot_diff_ckpt= str(self.get_parameter("slot_diff_ckpt").value)
        color_net_ckpt= str(self.get_parameter("color_net_ckpt").value)
        slot_device   = str(self.get_parameter("slot_device").value)
        self._execute = bool(self.get_parameter("execute").value)
        record_dir    = str(self.get_parameter("record_dir").value)
        self._pick_color  = str(self.get_parameter("pick_color").value)
        self._task_type   = str(self.get_parameter("task_type").value)
        self._target_color= str(self.get_parameter("target_color").value)

        # 모델 로드
        import sys
        sys.path.insert(0, str(_WS_ROOT / "src" / "ml"))
        from stable_baselines3 import PPO
        from mujoco_phase_rl.policies.train_ppo import _make_mixed_policy
        from mujoco_phase_rl.perception.image_embedding import SlotEmbedder

        self.get_logger().info(f"Loading PPO model: {model_path}")
        self._ppo = PPO.load(
            model_path, device="cpu",
            custom_objects={"policy_class": _make_mixed_policy()},
        )
        self.get_logger().info("Loading SlotEmbedder...")
        self._embedder = SlotEmbedder(
            stage1_ckpt=stage1_ckpt,
            slot_diff_ckpt=slot_diff_ckpt,
            color_net_ckpt=color_net_ckpt,
            device=slot_device,
        )

        # 녹화 설정
        self._record_dir: Path | None = None
        self._episode_buffer: list[dict[str, Any]] = []
        self._episode_count = 0
        if record_dir:
            self._record_dir = Path(record_dir)
            self._record_dir.mkdir(parents=True, exist_ok=True)

        # Phase 상태 추적
        self._phase_id: int = 0  # APPROACH
        self._step_count: int = 0
        self._prev_command_id: int = 0
        self._prev_result_id: int = 0
        self._prev_reward: float = 0.0
        self._history = np.zeros(13, dtype=np.float32)
        self._slot_diff = np.zeros(64, dtype=np.float32)

        # ROS 구독/발행
        self.create_subscription(Image, "/camera/camera/color/image_raw",
                                  self._on_image, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states",
                                  self._on_joints, 10)
        self.create_subscription(String, "/task_fsm/status",
                                  self._on_fsm_status, 10)
        self._status_pub = self.create_publisher(String, "/ppo_supervisor/status", 10)

        if self._execute:
            try:
                from msgs.msg import PickPlaceCommand
                self._cmd_pub = self.create_publisher(
                    PickPlaceCommand, "/pickplace/command", 10
                )
                self.get_logger().info("execute mode: /pickplace/command publisher ready")
            except ImportError:
                self.get_logger().error("msgs package not found — execute mode disabled")
                self._execute = False
                self._cmd_pub = None
        else:
            self._cmd_pub = None
            self.get_logger().info("shadow mode: actions computed but not published")

    def _on_image(self, msg: Image) -> None:
        try:
            img_bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return
        with self._lock:
            self._latest_img = img_bgr

    def _on_joints(self, msg: JointState) -> None:
        with self._lock:
            self._latest_joints = np.array(msg.position[:7], dtype=np.float32)

    def _on_fsm_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            with self._lock:
                self._fsm_status = str(data.get("state", "UNKNOWN"))
        except Exception:
            pass

    def _build_obs(self, img_bgr: np.ndarray, joints: np.ndarray) -> dict[str, np.ndarray]:
        """카메라 이미지 + 관절 상태 → PPO obs dict."""
        emb, curr_slots = self._embedder.embed_bgr(img_bgr)
        self._slot_diff = emb

        # robot obs (11-dim): joint_pos(6) + gripper(1) + ee_xyz(3) + grasped(1)
        robot = np.zeros(11, dtype=np.float32)
        robot[:min(7, len(joints))] = joints[:7]

        # task obs (4-dim): object_xy + target_xy from slot detection (zeros fallback)
        task = np.zeros(4, dtype=np.float32)
        if curr_slots and "xy" in curr_slots:
            present = curr_slots["present"][:, 0]  # (N,)
            xy = curr_slots["xy"]  # (N, 2)
            top_idx = int(np.argmax(present))
            task[:2] = xy[top_idx]  # object_xy estimate

        # phase obs (9-dim): one-hot
        phase = np.zeros(9, dtype=np.float32)
        phase[min(self._phase_id, 8)] = 1.0

        # cmd obs (9-dim): [task_type(2), obj_color(3), target(4)]
        task_oh = [1.0, 0.0] if self._task_type == "pick_place" else [0.0, 1.0]
        obj_oh  = [float(self._pick_color == c) for c in _BLOCK_COLORS]
        tgt_lbl = self._target_color if self._target_color != "basket" else "basket"
        tgt_oh  = [float(tgt_lbl == t) for t in _TARGETS]
        cmd = np.array(task_oh + obj_oh + tgt_oh, dtype=np.float32)

        return {
            "robot":    robot,
            "task":     task,
            "phase":    phase,
            "history":  self._history.copy(),
            "slot_diff": emb,
            "cmd":      cmd,
        }

    def step(self, task_pick_color: str, task_type: str, target_color: str) -> None:
        """한 PPO 스텝 실행 (shadow 또는 execute)."""
        with self._lock:
            img = self._latest_img.copy() if self._latest_img is not None else None
            joints = self._latest_joints.copy() if self._latest_joints is not None else None

        if img is None or joints is None:
            self.get_logger().warning("No camera/joint data yet")
            return

        obs = self._build_obs(img, joints)
        action, _ = self._ppo.predict(obs, deterministic=True)

        self.get_logger().info(
            f"PPO action: {action}  phase={self._phase_id}  execute={self._execute}"
        )

        if self._execute and self._cmd_pub is not None:
            self._publish_command(obs, action)

        if self._record_dir is not None:
            self._episode_buffer.append({
                "img_bgr": img,
                "obs":     {k: v.copy() for k, v in obs.items()},
                "action":  action.copy(),
                "phase_id": self._phase_id,
            })

        status = {
            "step": self._step_count,
            "phase": self._phase_id,
            "execute": self._execute,
            "fsm_status": self._fsm_status,
        }
        out = String(); out.data = json.dumps(status)
        self._status_pub.publish(out)
        self._step_count += 1

    def _publish_command(self, obs: dict, action: np.ndarray) -> None:
        """PPO action을 PickPlaceCommand로 변환해 publish."""
        from msgs.msg import PickPlaceCommand
        cmd = PickPlaceCommand()
        cmd.task = self._task_type
        # task obs에서 slot XY 사용
        cmd.x_pick  = float(obs["task"][0])
        cmd.y_pick  = float(obs["task"][1])
        cmd.yaw_pick = 0.0
        cmd.x_place = float(obs["task"][2])
        cmd.y_place = float(obs["task"][3])
        cmd.yaw_place = 0.0
        self._cmd_pub.publish(cmd)

    def save_episode(self, success: bool) -> None:
        """현재 에피소드 버퍼를 .npz로 저장."""
        if not self._episode_buffer or self._record_dir is None:
            return
        ep_id = self._episode_count
        path = self._record_dir / f"episode_{ep_id:05d}.npz"
        imgs  = np.stack([s["img_bgr"] for s in self._episode_buffer])
        obs_keys = list(self._episode_buffer[0]["obs"].keys())
        obs_arrays = {k: np.stack([s["obs"][k] for s in self._episode_buffer])
                      for k in obs_keys}
        actions = np.stack([s["action"] for s in self._episode_buffer])
        np.savez_compressed(
            path,
            imgs=imgs,
            actions=actions,
            success=np.array([success]),
            pick_color=np.array([self._pick_color]),
            task_type=np.array([self._task_type]),
            target_color=np.array([self._target_color]),
            **{f"obs_{k}": v for k, v in obs_arrays.items()},
        )
        manifest = self._record_dir / "episodes.json"
        entries = json.loads(manifest.read_text()) if manifest.exists() else []
        entries.append({"id": ep_id, "path": str(path), "success": success,
                        "steps": len(self._episode_buffer),
                        "pick_color": self._pick_color, "task_type": self._task_type})
        manifest.write_text(json.dumps(entries, indent=2))
        self._episode_count += 1
        self._episode_buffer.clear()
        self.get_logger().info(f"Episode saved: {path} (success={success})")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PPOSupervisorNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: setup.py entry_point 추가**

`mujoco_phase_rl/setup.py`에서 `console_scripts`에 추가:

```python
'ppo_supervisor = mujoco_phase_rl.ros2.ppo_supervisor_node:main',
```

- [ ] **Step 4: 빌드 확인**

```bash
cd ~/idle_ws
colcon build --symlink-install --packages-select mujoco_phase_rl 2>&1 | tail -5
source install/setup.bash
```

- [ ] **Step 5: import 확인**

```bash
python3 -c "from mujoco_phase_rl.ros2.ppo_supervisor_node import PPOSupervisorNode; print('OK')"
```

- [ ] **Step 6: 커밋**

```bash
git add mujoco_phase_rl/mujoco_phase_rl/ros2/ mujoco_phase_rl/setup.py
git commit -m "feat: ppo_supervisor_node — ROS2 shadow/execute mode + episode recording"
```

---

### Task 2: Real Val Pool Builder

녹화된 에피소드의 실이미지를 `_build_val_pool()`과 동일한 포맷으로 변환하는 스크립트.

**Files:**
- Create: `mujoco_phase_rl/mujoco_phase_rl/policies/build_real_val_pool.py`

**Interfaces:**
- Consumes: `record_dir/episodes.json` + `episode_NNNNN.npz`
- Produces: `real_val_pool.json` (scene별 task_sample 직렬화) + 이미지 캐시
- Produces: `real_pool_dir/` — `{episode_id:05d}.jpg` 이미지 + `pool.json`

- [ ] **Step 1: 스크립트 작성**

```python
# ================================================================
# build_real_val_pool.py
# 설명: ppo_supervisor_node가 녹화한 에피소드 → val_pool 포맷 변환.
#       결과 디렉토리를 finetune_from_real.py --real-pool-dir로 지정한다.
# 사용법:
#   python3 mujoco_phase_rl/policies/build_real_val_pool.py \
#     --record-dir /tmp/episodes --out-dir outputs/real_val_pool
# ================================================================
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--record-dir", required=True)
    parser.add_argument("--out-dir",    default="outputs/real_val_pool")
    args = parser.parse_args()

    record_dir = Path(args.record_dir)
    out_dir    = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    manifest = record_dir / "episodes.json"
    if not manifest.exists():
        raise SystemExit(f"episodes.json not found in {record_dir}")

    entries = json.loads(manifest.read_text())
    pool = []

    for entry in entries:
        npz_path = Path(entry["path"])
        if not npz_path.exists():
            print(f"[skip] {npz_path} not found")
            continue

        data = np.load(npz_path, allow_pickle=True)
        imgs   = data["imgs"]           # (T, H, W, 3) BGR
        pick_color   = str(data["pick_color"][0])
        task_type    = str(data["task_type"][0])
        target_color = str(data["target_color"][0])
        success      = bool(data["success"][0])

        # 첫 프레임을 val image로 사용 (물체 위치가 가장 안정적)
        first_img = imgs[0]
        img_path  = out_dir / f"{entry['id']:05d}.jpg"
        cv2.imwrite(str(img_path), first_img)

        # obs에서 task sample 복원
        obj_xy  = data["obs_task"][0, :2].tolist() if "obs_task" in data else [0.0, 0.0]
        tgt_xy  = data["obs_task"][0, 2:4].tolist() if "obs_task" in data else [0.0, 0.0]

        pool.append({
            "img_path":     str(img_path),
            "pick_color":   pick_color,
            "task_type":    task_type,
            "target_color": target_color if task_type == "stack" else None,
            "success":      success,
            "object_xy":    obj_xy,
            "target_xy":    tgt_xy,
            "episode_id":   entry["id"],
        })

    (out_dir / "pool.json").write_text(json.dumps(pool, indent=2))
    print(f"Real val pool: {len(pool)} entries → {out_dir}/pool.json")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 커밋**

```bash
git add mujoco_phase_rl/mujoco_phase_rl/policies/build_real_val_pool.py
git commit -m "feat: build_real_val_pool.py — 녹화 에피소드를 val_pool 포맷으로 변환"
```

---

### Task 3: finetune_from_real.py — 실데이터 Domain-Adaptive Fine-tuning

실이미지 기반 val_pool을 쓰는 fine-tuning 스크립트. `finetune_stack_robust.py`에서 val_pool 로딩 부분만 교체한다.

**Files:**
- Create: `mujoco_phase_rl/mujoco_phase_rl/policies/finetune_from_real.py`

**Interfaces:**
- Consumes: `--base-model` (sim-trained PPO zip), `--real-pool-dir` (build_real_val_pool 출력)
- Consumes: `finetune_stack_robust._build_vec_env()` (재사용)
- Produces: `--output-dir/final_model.zip` (실이미지 분포에 적응된 PPO)

- [ ] **Step 1: 스크립트 작성**

```python
# ================================================================
# finetune_from_real.py
# 설명: 실카메라 이미지로 구성된 val_pool로 PPO를 domain-adaptive fine-tuning.
#       ppo_supervisor_node 녹화 → build_real_val_pool → 이 스크립트 순서.
# 사용법:
#   python3 mujoco_phase_rl/policies/finetune_from_real.py \
#     --base-model outputs/ppo_stack_pg_s0/final_model.zip \
#     --real-pool-dir outputs/real_val_pool \
#     --output-dir outputs/ppo_real_adapted
# ================================================================
from __future__ import annotations

import argparse
import json
import random
from pathlib import Path

import cv2
import numpy as np


def _load_real_pool(pool_dir: Path) -> list[tuple]:
    """real val pool → [(task_sample, img_bgr), ...] (finetune_stack_robust과 동일 포맷)."""
    from mujoco_phase_rl.policies.run_val_sim import dets_to_task_sample

    pool_json = json.loads((pool_dir / "pool.json").read_text())
    pool = []
    for entry in pool_json:
        img = cv2.imread(entry["img_path"])
        if img is None:
            continue

        # pool.json의 좌표로 TaskSample 직접 구성
        from mujoco_phase_rl.tasks.pick_place_task import TaskSample
        pick_color   = entry["pick_color"]
        task_type    = entry["task_type"]
        target_color = entry.get("target_color")
        obj_xy  = entry.get("object_xy", [0.0, 0.0])
        tgt_xy  = entry.get("target_xy", [0.0, 0.0])

        _BLOCK_Z  = 0.023
        _STACK_Z  = 0.063
        _BASKET_Z = 0.009
        z_tgt = _STACK_Z if task_type == "stack" else _BASKET_Z

        ts = TaskSample(
            object_pos=np.array([obj_xy[0], obj_xy[1], _BLOCK_Z], dtype=np.float64),
            object_quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            target_pos=np.array([tgt_xy[0], tgt_xy[1], z_tgt], dtype=np.float64),
            target_yaw=0.0,
            object_mass=0.10,
            pick_color=pick_color,
            task_type=task_type,
            target_color=target_color,
        )
        pool.append((ts, img))
    return pool


def main() -> None:
    parser = argparse.ArgumentParser(description="실이미지 기반 domain-adaptive PPO fine-tuning")
    parser.add_argument("--base-model",   default="outputs/ppo_stack_pg_s0/final_model.zip")
    parser.add_argument("--real-pool-dir", required=True)
    parser.add_argument("--output-dir",   default="outputs/ppo_real_adapted")
    parser.add_argument("--total-timesteps", type=int, default=50_000)
    parser.add_argument("--n-envs",       type=int, default=4)
    parser.add_argument("--learning-rate", type=float, default=5e-5)
    parser.add_argument("--n-steps",      type=int, default=256)
    parser.add_argument("--batch-size",   type=int, default=512)
    parser.add_argument("--seed",         type=int, default=0)
    parser.add_argument("--slot-device",  default="cuda")
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    real_pool = _load_real_pool(Path(args.real_pool_dir))
    print(f"real val pool: {len(real_pool)} (task_sample, img) pairs")
    if not real_pool:
        raise SystemExit("real pool이 비어있음. build_real_val_pool.py 먼저 실행")

    # finetune_stack_robust의 _build_vec_env 재사용 (val_pool만 real로 교체)
    from mujoco_phase_rl.policies.finetune_stack_robust import (
        _build_vec_env, load_base_model,
        _DEFAULT_STAGE1, _DEFAULT_SLOT_DIFF, _DEFAULT_COLOR_NET,
    )
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import CheckpointCallback

    # args 호환 namespace 구성
    class FinetuneArgs:
        n_envs = args.n_envs
        seed   = args.seed
        max_episode_steps = 64
        stack_prob  = 0.6
        aug_prob    = 0.0   # 실이미지 자체가 real이므로 aug 불필요
        perturb_prob = 0.0
        perturb_max  = 0.05
        pose_source  = "slot"
        pose_noise_std = 0.0
        target_noise_std = 0.0
        pose_dropout_prob = 0.0
        slot_stage1_ckpt    = _DEFAULT_STAGE1
        slot_diff_ckpt      = _DEFAULT_SLOT_DIFF
        slot_color_net_ckpt = _DEFAULT_COLOR_NET
        slot_transition_ckpt= None
        slot_device         = args.slot_device
        no_command_mask     = False
        no_aug_slot         = True

    vec_env = _build_vec_env(FinetuneArgs(), real_pool)
    model = load_base_model(ppo_cls=PPO, base_model=args.base_model, env=vec_env, device="auto")
    model.learning_rate = args.learning_rate
    model.n_steps  = args.n_steps
    model.batch_size = min(args.batch_size, args.n_steps * args.n_envs)

    ckpt_cb = CheckpointCallback(
        save_freq=max(10240 // args.n_envs, 1),
        save_path=str(output_dir / "checkpoints"),
        name_prefix="ppo_real",
    )
    model.learn(total_timesteps=args.total_timesteps, callback=ckpt_cb, reset_num_timesteps=False)
    model.save(output_dir / "final_model.zip")
    vec_env.close()
    print(f"saved: {output_dir}/final_model.zip")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 커밋**

```bash
git add mujoco_phase_rl/mujoco_phase_rl/policies/finetune_from_real.py
git commit -m "feat: finetune_from_real.py — 실이미지 val_pool 기반 domain-adaptive fine-tuning"
```

---

### Task 4: 런치 파일 통합

데모 실행 시 ppo_supervisor를 기존 launch에 쉽게 붙일 수 있도록 launch 파일 추가.

**Files:**
- Create: `src/idle_launch/launch/ppo_demo.launch.py`

**Interfaces:**
- Launches: `can_bridge_node`, `pick_place_control.launch.py`, `ppo_supervisor_node`
- Args: `model_path`, `execute`, `record_dir`, `pick_color`, `task_type`, `target_color`

- [ ] **Step 1: launch 파일 작성**

```python
# src/idle_launch/launch/ppo_demo.launch.py
from __future__ import annotations
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model_path",    default_value="outputs/ppo_stack_pg_s0/final_model.zip"),
        DeclareLaunchArgument("execute",       default_value="false"),
        DeclareLaunchArgument("record_dir",    default_value=""),
        DeclareLaunchArgument("pick_color",    default_value="red"),
        DeclareLaunchArgument("task_type",     default_value="pick_place"),
        DeclareLaunchArgument("target_color",  default_value="basket"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare("idle_launch"), "launch",
                                      "pick_place_control.launch.py"])
            ])
        ),

        Node(
            package="mujoco_phase_rl",
            executable="ppo_supervisor",
            name="ppo_supervisor_node",
            parameters=[{
                "model_path":    LaunchConfiguration("model_path"),
                "execute":       LaunchConfiguration("execute"),
                "record_dir":    LaunchConfiguration("record_dir"),
                "pick_color":    LaunchConfiguration("pick_color"),
                "task_type":     LaunchConfiguration("task_type"),
                "target_color":  LaunchConfiguration("target_color"),
            }],
            output="screen",
        ),
    ])
```

- [ ] **Step 2: 빌드 및 확인**

```bash
cd ~/idle_ws
colcon build --symlink-install --packages-select idle_launch mujoco_phase_rl 2>&1 | tail -5
source install/setup.bash
ros2 launch idle_launch ppo_demo.launch.py --show-args
```

- [ ] **Step 3: 커밋**

```bash
git add src/idle_launch/launch/ppo_demo.launch.py
git commit -m "feat: ppo_demo.launch.py — supervisor + FSM 통합 런치"
```

---

## 실행 순서 요약

```
# 1. 학습 완료 후 eval
python3 mujoco_phase_rl/policies/run_val_sim_batch.py \
    --max-scenes 20 --out outputs/ppo_stack_pg_s0/eval_val20_slot.json

# 2. 데모 (shadow mode — 실행 없이 recording만)
ros2 launch idle_launch ppo_demo.launch.py \
    record_dir:=/tmp/episodes execute:=false

# 3. execute mode 전환 (검증 후)
ros2 launch idle_launch ppo_demo.launch.py \
    execute:=true record_dir:=/tmp/episodes

# 4. 실데이터 pool 빌드
python3 mujoco_phase_rl/policies/build_real_val_pool.py \
    --record-dir /tmp/episodes --out-dir outputs/real_val_pool

# 5. 실이미지 fine-tuning
python3 mujoco_phase_rl/policies/finetune_from_real.py \
    --real-pool-dir outputs/real_val_pool \
    --output-dir outputs/ppo_real_adapted
```
