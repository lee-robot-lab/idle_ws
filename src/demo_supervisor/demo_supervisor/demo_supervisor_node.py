# ================================================================
# demo_supervisor/demo_supervisor_node.py
# 설명: STT/Qwen → Stage1/2/4 grounding → PickPlaceCommand publish.
#       단일 프로세스 supervisor. ML tensor는 토픽으로 내보내지 않는다.
# 사용법:
#   ros2 run demo_supervisor demo_supervisor_node \
#       --ros-args -p stage1_ckpt:=<path> -p text:='파란 블록을 바구니에 넣어줘'
# ================================================================
from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional

import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from msgs.msg import PickPlaceCommand
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from demo_supervisor.ml.pipeline import GroundingResult, MLPipeline
from demo_supervisor.ml.stt_client import parse_text


# ── 기본 체크포인트 경로 ────────────────────────────────────────────
import os
_WS = os.path.join(os.path.dirname(__file__), *(['..'] * 4))
_CKPT = os.path.join(_WS, 'checkpoints')

_DEFAULT_STAGE1     = os.path.join(_CKPT, 'stage1_v2', 'best.pt')
_DEFAULT_COLOR_NET  = os.path.join(_CKPT, 'color_net_v2', 'best.pt')
_DEFAULT_STAGE4     = os.path.join(_CKPT, 'stage4', 'best.pt')
_DEFAULT_SLOT_DIFF  = os.path.join(_CKPT, 'slot_diff', 'best.pt')
_DEFAULT_PPO        = os.path.join(_WS, 'src', 'mujoco_phase_rl',
                                   'outputs', 'ppo_stack_base_s0', 'final_model.zip')


# ── Supervisor 상태 머신 ────────────────────────────────────────────
class State(Enum):
    IDLE       = auto()
    CAPTURE    = auto()
    GROUND     = auto()
    PUBLISH    = auto()
    WAIT_FSM   = auto()
    ERROR      = auto()


@dataclass
class SupervisorContext:
    """worker thread가 공유하는 가변 상태. GIL 보호로 충분."""
    state:          State = State.IDLE
    current_text:   str = ""
    current_parser: str = "rule"
    latest_frame:   Optional[np.ndarray] = None
    frame_stamp_s:  float = 0.0
    fsm_state:      str = "IDLE"
    drop_detected:  bool = False
    last_error:     str = ""
    command_result: Optional[GroundingResult] = None


# ── 안전 게이트 ─────────────────────────────────────────────────────
_WORKSPACE = dict(x_min=-0.1, x_max=0.5, y_min=0.1, y_max=0.8)
_MIN_CONFIDENCE = 0.3
_MAX_FRAME_AGE_S = 3.0


def _safety_check(gr: GroundingResult, task_type: str) -> tuple[bool, str]:
    if not (math.isfinite(gr.x_pick) and math.isfinite(gr.y_pick)
            and math.isfinite(gr.x_place) and math.isfinite(gr.y_place)
            and math.isfinite(gr.yaw_pick) and math.isfinite(gr.yaw_place)):
        return False, "non-finite value in grounding result"

    ws = _WORKSPACE
    for label, x, y in [("pick", gr.x_pick, gr.y_pick), ("place", gr.x_place, gr.y_place)]:
        if not (ws["x_min"] <= x <= ws["x_max"] and ws["y_min"] <= y <= ws["y_max"]):
            return False, f"{label} ({x:.3f}, {y:.3f}) outside workspace"

    if gr.confidence < _MIN_CONFIDENCE:
        return False, f"grounding confidence {gr.confidence:.2f} < {_MIN_CONFIDENCE}"

    pick_pos = (round(gr.x_pick, 3), round(gr.y_pick, 3))
    place_pos = (round(gr.x_place, 3), round(gr.y_place, 3))
    if pick_pos == place_pos:
        return False, "pick and place position identical"

    return True, "ok"


# ── 메인 노드 ───────────────────────────────────────────────────────
class DemoSupervisorNode(Node):
    """단일 supervisor 노드. camera/FSM 구독 + worker thread에서 ML 추론."""

    def __init__(self) -> None:
        super().__init__('demo_supervisor')

        # 파라미터
        self.declare_parameter('stage1_ckpt',    _DEFAULT_STAGE1)
        self.declare_parameter('color_net_ckpt', _DEFAULT_COLOR_NET)
        self.declare_parameter('stage4_ckpt',    _DEFAULT_STAGE4)
        self.declare_parameter('device',         'cpu')
        self.declare_parameter('camera_topic',   '/image_raw')
        self.declare_parameter('parser',         'rule')
        self.declare_parameter('debug_image',    False)
        self.declare_parameter('save_dir',       '')
        self.declare_parameter('text',           '')  # 노드 시작 시 한 번 실행할 텍스트
        self.declare_parameter('ppo_mode',       False)
        self.declare_parameter('ppo_task_topic', '/ppo/task')
        self.declare_parameter('ppo_done_topic', '/ppo/done')

        p = lambda n: self.get_parameter(n).value  # noqa: E731

        # ML 파이프라인 (시작 시 1회 로드)
        self.get_logger().info('Loading ML models…')
        self._pipeline = MLPipeline(
            stage1_ckpt=p('stage1_ckpt'),
            color_net_ckpt=p('color_net_ckpt'),
            stage4_ckpt=p('stage4_ckpt'),
            device=p('device'),
        )
        self.get_logger().info('ML models loaded.')

        self._ctx = SupervisorContext()
        self._bridge = CvBridge()
        self._worker_lock = threading.Lock()

        raw_save = p('save_dir')
        self._save_dir: Path | None = (
            Path(raw_save).expanduser() if raw_save else None
        )

        # ROS 인터페이스
        self._cmd_pub    = self.create_publisher(PickPlaceCommand, '/pickplace/command', 10)
        self._status_pub = self.create_publisher(String, '/demo/status', 10)

        self.create_subscription(Image,  p('camera_topic'),     self._on_image,      10)
        self.create_subscription(String, '/task_fsm/status',    self._on_fsm_status,  10)
        self.create_subscription(Bool,   '/gripper/drop_detected', self._on_drop,     10)

        # PPO 모드
        self._ppo_mode = p('ppo_mode')
        if self._ppo_mode:
            self._ppo_task_pub = self.create_publisher(
                String, p('ppo_task_topic'), 10
            )
            self._ppo_done_event = threading.Event()
            self._ppo_done_result: dict = {}
            self.create_subscription(
                String, p('ppo_done_topic'), self._on_ppo_done, 10
            )
        else:
            self._ppo_task_pub = None
            self._ppo_done_event = None

        # 텍스트 명령 토픽 (외부에서 pub 가능)
        self.create_subscription(String, '/demo/command', self._on_command_topic, 10)

        if p('debug_image'):
            self._debug_pub = self.create_publisher(Image, '/demo/debug_image', 1)
        else:
            self._debug_pub = None

        # 시작 시 --ros-args -p text:=... 로 바로 실행
        startup_text = p('text')
        if startup_text:
            threading.Thread(
                target=self._run_command,
                args=(startup_text, p('parser')),
                daemon=True,
            ).start()

        self.get_logger().info(
            f"demo_supervisor ready  camera={p('camera_topic')}  device={p('device')}"
        )

    # ── ROS callbacks (lightweight, no PyTorch) ──────────────────────

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._ctx.latest_frame = frame
            self._ctx.frame_stamp_s = time.monotonic()
        except Exception as e:
            self.get_logger().warn(f'image convert error: {e}')

    def _on_fsm_status(self, msg: String) -> None:
        self._ctx.fsm_state = msg.data

    def _on_drop(self, msg: Bool) -> None:
        self._ctx.drop_detected = bool(msg.data)

    def _on_ppo_done(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._ppo_done_result = data
        if self._ppo_done_event is not None:
            self._ppo_done_event.set()

    def _on_command_topic(self, msg: String) -> None:
        """'/demo/command' 토픽으로 텍스트 명령 수신."""
        if self._ctx.state != State.IDLE:
            self._publish_status('busy', 'FSM or supervisor busy, command ignored')
            return
        text = msg.data.strip()
        if not text:
            return
        threading.Thread(
            target=self._run_command,
            args=(text, self.get_parameter('parser').value),
            daemon=True,
        ).start()

    # ── Worker (ML 추론은 여기서만) ──────────────────────────────────

    def _run_command(self, text: str, parser: str) -> None:
        """IDLE → CAPTURE → GROUND → PUBLISH → WAIT_FSM → IDLE."""
        if not self._worker_lock.acquire(blocking=False):
            self._publish_status('busy', 'another command in progress')
            return
        try:
            self._transition(State.CAPTURE)

            # STT 파싱
            step = parse_text(text, parser=parser)
            if step is None:
                self._error(f'STT parse failed: "{text}"')
                return

            # 카메라 프레임 스냅샷
            frame = self._ctx.latest_frame
            age = time.monotonic() - self._ctx.frame_stamp_s
            if frame is None or age > _MAX_FRAME_AGE_S:
                self._error(f'no fresh camera frame (age={age:.1f}s)')
                return
            self._save_debug_frame('capture', frame, {'text': text, 'parser': parser, 'frame_age_s': age})

            # Stage1/2/4 grounding
            self._transition(State.GROUND)
            gr = self._pipeline.ground(frame, step)
            if gr is None:
                self._save_debug_frame('ground_fail', frame, {'text': text, 'reason': 'object or target not found'})
                self._error('grounding failed: object or target not found')
                return

            # 안전 게이트
            ok, reason = _safety_check(gr, gr.task_type)
            if not ok:
                self._save_debug_frame('safety_fail', frame, {'text': text, 'reason': reason})
                self._error(f'safety gate: {reason}')
                return

            self._save_debug_frame('ground_ok', frame, {
                'text': text,
                'task': gr.task_type,
                'pick': [gr.x_pick, gr.y_pick, gr.yaw_pick],
                'place': [gr.x_place, gr.y_place, gr.yaw_place],
                'confidence': gr.confidence,
            })

            # FSM IDLE 확인
            if self._ctx.fsm_state != 'IDLE':
                self._error(f'FSM not IDLE (state={self._ctx.fsm_state})')
                return

            # Publish
            self._transition(State.PUBLISH)
            if self._ppo_mode and self._ppo_task_pub is not None:
                self._ppo_done_event.clear()
                self._ppo_done_result = {}
                self._publish_ppo_task(gr)
                self._publish_status('ppo_task_sent', json.dumps({
                    'text': text,
                    'object_pos': [gr.x_pick, gr.y_pick],
                    'target_pos': [gr.x_place, gr.y_place],
                }))

                # PPO 완료 대기
                self._transition(State.WAIT_FSM)
                done = self._ppo_done_event.wait(timeout=120.0)
                if not done:
                    self._error('PPO task timeout (120s)')
                    return
                result = self._ppo_done_result
                if not result.get('success'):
                    self._error(f"PPO task failed: phase={result.get('phase')} reason={result.get('reason', '')}")
                    return
                self._publish_status('ppo_done', json.dumps(result))
            else:
                self._publish_command(gr)
                self._publish_status('published', json.dumps({
                    'text': text,
                    'task': gr.task_type,
                    'pick': [gr.x_pick, gr.y_pick, gr.yaw_pick],
                    'place': [gr.x_place, gr.y_place, gr.yaw_place],
                    'confidence': gr.confidence,
                }))

                # FSM 완료 대기
                self._transition(State.WAIT_FSM)
                self._wait_fsm_done(timeout_s=60.0)

        finally:
            self._transition(State.IDLE)
            self._worker_lock.release()

    def _wait_fsm_done(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            s = self._ctx.fsm_state
            if s in ('IDLE', 'DONE', 'FAIL'):
                return
            time.sleep(0.1)
        self.get_logger().warn('FSM wait timeout')

    # ── 내부 유틸 ────────────────────────────────────────────────────

    def _transition(self, state: State) -> None:
        self._ctx.state = state
        self.get_logger().info(f'→ {state.name}')

    def _error(self, reason: str) -> None:
        self._ctx.last_error = reason
        self._transition(State.ERROR)
        self._publish_status('error', reason)
        self.get_logger().error(reason)

    def _publish_command(self, gr: GroundingResult) -> None:
        msg = PickPlaceCommand()
        msg.task     = gr.task_type
        msg.x_pick   = gr.x_pick
        msg.y_pick   = gr.y_pick
        msg.yaw_pick = gr.yaw_pick
        msg.x_place  = gr.x_place
        msg.y_place  = gr.y_place
        msg.yaw_place = gr.yaw_place
        self._cmd_pub.publish(msg)

    def _publish_ppo_task(self, gr: GroundingResult) -> None:
        """PPO 모드: grounding 결과를 /ppo/task로 발행."""
        msg = String()
        msg.data = json.dumps({
            "object_pos": [gr.x_pick, gr.y_pick, 0.009],
            "target_pos": [gr.x_place, gr.y_place, 0.009],
            "object_color": gr.pick_color,
            "task_type": gr.task_type,
        })
        self._ppo_task_pub.publish(msg)

    def _save_debug_frame(
        self,
        tag: str,
        frame: np.ndarray,
        meta: dict,
    ) -> None:
        """tag 이름으로 이미지와 JSON을 save_dir/<timestamp>/ 에 저장."""
        if self._save_dir is None:
            return
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        out = self._save_dir / ts
        out.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(out / f'{tag}.jpg'), frame)
        (out / f'{tag}.json').write_text(
            json.dumps(meta, indent=2, ensure_ascii=False)
        )
        self.get_logger().info(f'[debug] saved {out / tag}')

    def _publish_status(self, status: str, detail: str = '') -> None:
        msg = String()
        msg.data = json.dumps({'status': status, 'detail': detail}, ensure_ascii=False)
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DemoSupervisorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
