"""tick_monitor — 제어 토픽 케이던스·지터 진단 (라이브, 측정-우선).

[무엇]
  plan_node(/motor_cmd_array) 와 can_bridge(/motor_state_array) 의 publish 간격을
  받는 쪽에서 측정해 토픽별 주기 통계(p50/p90/p99/max)와 gap 횟수를 보고한다.
  "250Hz 제어 루프 지터가 실제로 문제인지"를 최적화 전에 먼저 확인하는 용도.

[요구사항]
  - 시스템이 가동 중이어야 함 (plan_node·can_bridge 가 토픽을 publish 하는 상태).
  - rclpy 기반 → ROS 환경 source 필요 (CAN-직결인 다른 motor/ 툴과 다름):
        source ~/idle_ws/install/setup.bash
  - python3 + numpy + msgs 패키지.

[사용]
  source ~/idle_ws/install/setup.bash
  cd ~/idle_ws/motor
  python3 tick_monitor.py --duration 20        # 20초 측정 후 자동 종료
  python3 tick_monitor.py --topics cmd          # 명령 토픽만, Ctrl-C 종료
  python3 tick_monitor.py --report 5            # 5초마다 중간 보고
  python3 tick_monitor.py --gap_factor 1.8      # 중앙값의 1.8배 초과 간격을 gap 집계

[출력 읽는 법]  예: dt[ms] p50=4.01 p90=4.3 p99=6.8 max=18 gaps>2x=5
  p50  ≈ 4.0ms 여야 정상(250Hz). 4보다 크면 실제 레이트가 250Hz 미달.
  p99·max 가 p50 대비 크게 벌어지면 = 지터 꼬리(GC·스케줄링).
  gaps = 정상 주기의 gap_factor 배 넘게 늦은 횟수. 0 에 가까워야 좋음.

[한계]
  여기서 보는 건 '전송선상 케이던스'(DDS 전송 지터 포함). plan_node on_timer 의
  *내부* 주기·GC 멈춤을 정밀히 보려면 노드 내부 계측(perf_counter + gc.get_stats)이
  더 정확하다 (이 툴은 비침습적 1차 점검용).
"""

import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from msgs.msg import MotorCMDArray, MotorStateArray

CMD_TOPIC = "/motor_cmd_array"
STATE_TOPIC = "/motor_state_array"


class TickMonitor(Node):
    def __init__(self, topics: list[str], report_s: float, gap_factor: float):
        super().__init__("tick_monitor")
        self.gap_factor = gap_factor
        self.samples: dict[str, list[float]] = {}  # topic → dt[ms] 리스트
        self.last: dict[str, float] = {}           # topic → 직전 도착 perf_counter

        be1 = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        be5 = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)

        if "cmd" in topics:
            self._arm(CMD_TOPIC)
            self.create_subscription(MotorCMDArray, CMD_TOPIC,
                                     lambda _m: self._on(CMD_TOPIC), be1)
        if "state" in topics:
            self._arm(STATE_TOPIC)
            self.create_subscription(MotorStateArray, STATE_TOPIC,
                                     lambda _m: self._on(STATE_TOPIC), be5)

        if report_s > 0:
            self.create_timer(report_s, self.report)
        self.get_logger().info(f"tick_monitor watching: {list(self.samples)}")

    def _arm(self, topic: str) -> None:
        self.samples[topic] = []
        self.last[topic] = None  # type: ignore[assignment]

    def _on(self, topic: str) -> None:
        t = time.perf_counter()
        prev = self.last[topic]
        self.last[topic] = t
        if prev is not None:
            self.samples[topic].append((t - prev) * 1.0e3)

    def report(self) -> None:
        for topic, s in self.samples.items():
            if len(s) < 2:
                self.get_logger().info(f"{topic}: <2 samples")
                continue
            a = np.asarray(s)
            med = float(np.median(a))
            gaps = int((a > self.gap_factor * med).sum())
            self.get_logger().info(
                f"{topic}: n={len(a)} rate={1000.0 / float(np.mean(a)):.1f}Hz "
                f"dt[ms] p50={med:.2f} p90={float(np.percentile(a, 90)):.2f} "
                f"p99={float(np.percentile(a, 99)):.2f} max={float(a.max()):.2f} "
                f"gaps>{self.gap_factor:g}x={gaps}"
            )


def main() -> None:
    ap = argparse.ArgumentParser(description="Measure /motor_cmd_array & /motor_state_array cadence jitter.")
    ap.add_argument("--topics", nargs="+", default=["cmd", "state"], choices=["cmd", "state"])
    ap.add_argument("--duration", type=float, default=0.0, help="초 단위, 0=무한(Ctrl-C 종료)")
    ap.add_argument("--report", type=float, default=5.0, help="주기 보고 간격(초), 0=주기보고 끔")
    ap.add_argument("--gap_factor", type=float, default=2.0, help="중앙값 대비 이 배수 초과 간격을 gap 으로 집계")
    args = ap.parse_args()

    rclpy.init()
    node = TickMonitor(args.topics, args.report, args.gap_factor)
    try:
        if args.duration > 0:
            end = time.perf_counter() + args.duration
            while rclpy.ok() and time.perf_counter() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("=== final ===")
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
