"""Pick-and-place task FSM node.

Command input:
    ros2 topic pub --once /pickplace/command msgs/msg/PickPlaceCommand \
        '{task: "stack", x_pick: 0.30, y_pick: 0.0, yaw_pick: 0.0, x_place: 0.30, y_place: 0.20, yaw_place: 0.0}'

    task: "" or "default" → ROS param z values
    task: "stack"  → block stacking preset (low z_place)
    task: "place"  → basket/place preset (higher z_pregrasp)
    Custom presets loaded from task_presets_yaml_path param.

Z constants (tunable via ROS params, used as defaults when task="" or "default"):
    z_pregrasp  = 0.40  -- approach / lift / transit height
    z_grasp     = 0.12  -- grasp descent
    z_place     = 0.20  -- place descent

State machine: -- 0.3 0.0 0.6 45
    IDLE -> PRE_GRASP -> GRASP_DESCEND -> GRASP_CLOSE
         -> LIFT -> TRANSIT -> PLACE_DESCEND
         -> GRASP_OPEN -> RETRACT -> HOME -> DONE -> IDLE
    Recoverable failure -> FAIL -> HOME -> IDLE
    Unsafe recovery failure -> FAULT (manual intervention / node restart)
"""

from __future__ import annotations

import math
from enum import Enum, auto
from typing import Optional, Sequence

import yaml

import rclpy
from geometry_msgs.msg import PoseStamped
from idle_common.ros_params import declare_typed
from msgs.msg import EETarget, PickPlaceCommand
from phy.task_validation import is_known_task
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class FSMState(Enum):
    IDLE = auto()
    PRE_GRASP = auto()
    GRASP_DESCEND = auto()
    GRASP_CLOSE = auto()
    LIFT = auto()
    TRANSIT = auto()
    PLACE_DESCEND = auto()
    GRASP_OPEN = auto()
    RETRACT = auto()
    HOME = auto()
    DONE = auto()
    FAIL = auto()
    FAULT = auto()


def _yaw_quat(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class TaskFSMNode(Node):
    TIMER_PERIOD_S = 0.1

    def __init__(self) -> None:
        super().__init__("task_fsm_node")

        self._z_pregrasp = float(declare_typed(self, "z_pregrasp", 0.40))
        self._z_grasp = float(declare_typed(self, "z_grasp", 0.12))
        self._z_place = float(declare_typed(self, "z_place", 0.20))
        self._grasp_descend_duration = float(
            declare_typed(self, "grasp_descend_duration_s", 2.0)
        )
        self._place_descend_duration = float(
            declare_typed(self, "place_descend_duration_s", 2.0)
        )
        self._post_grasp_hold_s = float(declare_typed(self, "post_grasp_hold_s", 0.4))
        # Defaults preserved for reset when task="" or "default"
        self._z_pregrasp_default = self._z_pregrasp
        self._z_grasp_default = self._z_grasp
        self._z_place_default = self._z_place
        self._grasp_descend_duration_default = self._grasp_descend_duration
        self._place_descend_duration_default = self._place_descend_duration
        self._post_grasp_hold_default = self._post_grasp_hold_s

        task_presets_yaml_path = declare_typed(self, "task_presets_yaml_path", "", cast=str)
        self._task_presets: dict = self._load_task_presets(task_presets_yaml_path)

        # Per-phase extra dwell after plan_node reports DONE (s). plan_node already
        # settles (joint+vel tol) before DONE; these add task-level margin where it
        # matters (hold steady before closing/opening the gripper).
        self._dwell_pre_grasp_s = float(declare_typed(self, "dwell_pre_grasp_s", 0.0))
        self._dwell_grasp_s = float(declare_typed(self, "dwell_grasp_s", 0.5))
        self._dwell_lift_s = float(declare_typed(self, "dwell_lift_s", 0.0))
        self._dwell_transit_s = float(declare_typed(self, "dwell_transit_s", 0.0))
        self._dwell_place_s = float(declare_typed(self, "dwell_place_s", 0.5))
        self._dwell_retract_s = float(declare_typed(self, "dwell_retract_s", 0.0))
        self._dwell_home_s = float(declare_typed(self, "dwell_home_s", 0.0))

        self._motion_timeout_s = float(declare_typed(self, "motion_timeout_s", 15.0))
        self._linear_motion_timeout_s = float(
            declare_typed(self, "linear_motion_timeout_s", 8.0)
        )
        self._home_timeout_s = float(declare_typed(self, "home_timeout_s", 15.0))
        self._service_timeout_s = float(declare_typed(self, "service_timeout_s", 2.0))

        self._x_min = float(declare_typed(self, "x_min", -1.0))
        self._x_max = float(declare_typed(self, "x_max", 1.0))
        self._y_min = float(declare_typed(self, "y_min", -1.0))
        self._y_max = float(declare_typed(self, "y_max", 1.0))

        self._state = FSMState.IDLE
        self._state_ticks: int = 0
        self._done_tick: int = 0
        self._plan_status: str = "IDLE"
        self._waiting_for_plan: bool = False
        self._plan_started: bool = False
        self._plan_wait_start_s: float = 0.0
        self._service_future = None
        self._service_start_s: float = 0.0
        self._service_name: str = ""
        self._grasp_success: Optional[bool] = None
        self._drop_detected: bool = False
        self._home_accepted: bool = False
        self._gripper_close_accepted: bool = False
        self._gripper_open_accepted: bool = False
        self._grasp_success_start_s: float = 0.0

        # Pick / place pose (set on command)
        self._pick_x = self._pick_y = self._pick_yaw = 0.0
        self._place_x = self._place_y = self._place_yaw = 0.0

        self.create_subscription(
            PickPlaceCommand, "/pickplace/command", self._on_command, 10
        )
        self.create_subscription(String, "/plan/status", self._on_plan_status, 10)
        self.create_subscription(Bool, "/gripper/grasp_success", self._on_grasp_success, 10)
        self.create_subscription(Bool, "/gripper/drop_detected", self._on_drop_detected, 10)

        self._ee_pub = self.create_publisher(EETarget, "/ee_target", 10)
        self._status_pub = self.create_publisher(String, "/task_fsm/status", 10)

        self._gripper_open = self.create_client(Trigger, "/gripper/open")
        self._gripper_close = self.create_client(Trigger, "/gripper/close")
        self._go_home = self.create_client(Trigger, "/go_home")

        if not self._validate_startup_params():
            self._state = FSMState.FAULT

        self.create_timer(self.TIMER_PERIOD_S, self._on_timer)
        self.create_timer(1.0, self._publish_fsm_status)
        self._publish_fsm_status()

        self.get_logger().info(
            f"task_fsm_node ready -- publish msgs/PickPlaceCommand to /pickplace/command "
            f"(task: {list(self._task_presets.keys()) or 'none loaded'})"
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_command(self, msg: PickPlaceCommand) -> None:
        if self._state != FSMState.IDLE:
            self.get_logger().warn("busy or faulted -- ignoring command")
            return
        values = [msg.x_pick, msg.y_pick, msg.yaw_pick, msg.x_place, msg.y_place, msg.yaw_place]
        if not self._valid_command(values):
            return

        task = (msg.task or "").strip()
        if not is_known_task(task, self._task_presets):
            self.get_logger().error(f"unknown task '{task}' -- rejecting command")
            return
        self._apply_task_preset(task)

        self._pick_x, self._pick_y, self._pick_yaw = msg.x_pick, msg.y_pick, msg.yaw_pick
        self._place_x, self._place_y, self._place_yaw = msg.x_place, msg.y_place, msg.yaw_place
        self.get_logger().info(
            f"task={task or 'default'} "
            f"pick=({self._pick_x:.3f}, {self._pick_y:.3f}, yaw={self._pick_yaw:.3f}) "
            f"place=({self._place_x:.3f}, {self._place_y:.3f}, yaw={self._place_yaw:.3f})"
        )
        self._transition(FSMState.PRE_GRASP)

    def _on_plan_status(self, msg: String) -> None:
        status = msg.data.strip()
        self._plan_status = status
        if self._waiting_for_plan and status in ("PLANNING", "EXECUTING"):
            self._plan_started = True

    def _on_grasp_success(self, msg: Bool) -> None:
        self._grasp_success = bool(msg.data)

    def _on_drop_detected(self, msg: Bool) -> None:
        if msg.data:
            self._drop_detected = True

    # ------------------------------------------------------------------
    # FSM
    # ------------------------------------------------------------------

    def _on_timer(self) -> None:
        self._state_ticks += 1
        s = self._state

        if s == FSMState.PRE_GRASP:
            if self._state_ticks == 1:
                self._move(self._pick_x, self._pick_y, self._z_pregrasp, self._pick_yaw)
            else:
                self._advance_after_done(
                    FSMState.GRASP_DESCEND, self._dwell_pre_grasp_s, self._motion_timeout_s
                )

        elif s == FSMState.GRASP_DESCEND:
            if self._state_ticks == 1:
                self._move(
                    self._pick_x, self._pick_y, self._z_grasp, self._pick_yaw,
                    duration_s=self._grasp_descend_duration,
                    straight_line=True,
                )
            else:
                self._advance_after_done(
                    FSMState.GRASP_CLOSE, self._dwell_grasp_s, self._linear_motion_timeout_s
                )

        elif s == FSMState.GRASP_CLOSE:
            if self._state_ticks == 1:
                self._grasp_success = None
                if not self._start_gripper_call(close=True, fail_state=FSMState.FAIL):
                    return
            elif not self._gripper_close_accepted:
                if self._service_succeeded():
                    self._gripper_close_accepted = True
                elif self._service_failed_or_timed_out():
                    self._transition(FSMState.FAIL)
            elif self._grasp_success is True:
                if self._grasp_success_start_s <= 0.0:
                    self._grasp_success_start_s = self._now_s()
                    self.get_logger().info(
                        f"grasp success; holding {self._post_grasp_hold_s:.2f}s before lift"
                    )
                elif self._now_s() - self._grasp_success_start_s >= self._post_grasp_hold_s:
                    self._transition(FSMState.LIFT)
            elif self._grasp_success is False:
                self.get_logger().error("grasp failed")
                self._transition(FSMState.FAIL)

        elif s == FSMState.LIFT:
            if self._state_ticks == 1:
                self._drop_detected = False
                self._move(
                    self._pick_x, self._pick_y, self._z_pregrasp, self._pick_yaw,
                    straight_line=True,
                )
            elif self._drop_detected:
                self.get_logger().warn("drop detected during lift")
                self._transition(FSMState.FAIL)
            else:
                self._advance_after_done(
                    FSMState.TRANSIT, self._dwell_lift_s, self._linear_motion_timeout_s
                )

        elif s == FSMState.TRANSIT:
            if self._state_ticks == 1:
                self._move(self._place_x, self._place_y, self._z_pregrasp, self._place_yaw)
            elif self._drop_detected:
                self.get_logger().warn("drop detected during transit")
                self._transition(FSMState.FAIL)
            else:
                self._advance_after_done(
                    FSMState.PLACE_DESCEND, self._dwell_transit_s, self._motion_timeout_s
                )

        elif s == FSMState.PLACE_DESCEND:
            if self._state_ticks == 1:
                self._move(
                    self._place_x, self._place_y, self._z_place, self._place_yaw,
                    duration_s=self._place_descend_duration,
                    straight_line=True,
                )
            else:
                self._advance_after_done(
                    FSMState.GRASP_OPEN, self._dwell_place_s, self._linear_motion_timeout_s
                )

        elif s == FSMState.GRASP_OPEN:
            if self._state_ticks == 1:
                if not self._start_gripper_call(close=False, fail_state=FSMState.FAIL):
                    return
            elif not self._gripper_open_accepted:
                if self._service_succeeded():
                    self._gripper_open_accepted = True
                elif self._service_failed_or_timed_out():
                    self._transition(FSMState.FAIL)
            elif self._state_ticks >= 5:
                self._transition(FSMState.RETRACT)

        elif s == FSMState.RETRACT:
            if self._state_ticks == 1:
                self._move(
                    self._place_x, self._place_y, self._z_pregrasp, self._place_yaw,
                    straight_line=True,
                )
            else:
                self._advance_after_done(
                    FSMState.HOME, self._dwell_retract_s, self._linear_motion_timeout_s
                )

        elif s == FSMState.HOME:
            if self._state_ticks == 1:
                self._begin_plan_wait()
                self._home_accepted = False
                if not self._start_service_call(
                    self._go_home, "go_home", fail_state=FSMState.FAULT
                ):
                    return
            elif not self._home_accepted:
                if self._service_succeeded():
                    self._home_accepted = True
                    self._plan_started = True
                elif self._service_failed_or_timed_out():
                    self._transition(FSMState.FAULT)
            else:
                self._advance_after_done(
                    FSMState.DONE, self._dwell_home_s, self._home_timeout_s,
                    fail_state=FSMState.FAULT,
                )

        elif s == FSMState.DONE:
            self.get_logger().info("pick-and-place complete")
            self._transition(FSMState.IDLE)

        elif s == FSMState.FAIL:
            if self._state_ticks == 1:
                self.get_logger().error("task FAIL -- gripper open requested, returning home")
                if not self._start_gripper_call(close=False, fail_state=FSMState.FAULT):
                    return
            elif self._service_succeeded():
                self._transition(FSMState.HOME)
            elif self._service_failed_or_timed_out():
                self._transition(FSMState.FAULT)

        elif s == FSMState.FAULT:
            pass

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _transition(self, new: FSMState) -> None:
        self.get_logger().info(f"FSM: {self._state.name} -> {new.name}")
        self._state = new
        self._state_ticks = 0
        self._done_tick = 0
        self._plan_status = "IDLE"
        self._waiting_for_plan = False
        self._plan_started = False
        self._plan_wait_start_s = 0.0
        self._service_future = None
        self._service_start_s = 0.0
        self._service_name = ""
        self._home_accepted = False
        self._gripper_close_accepted = False
        self._gripper_open_accepted = False
        self._grasp_success_start_s = 0.0
        self._publish_fsm_status()

    def _plan_done(self) -> bool:
        return self._waiting_for_plan and self._plan_started and self._plan_status == "DONE"

    def _advance_after_done(
        self,
        nxt: FSMState,
        dwell_s: float,
        timeout_s: float,
        fail_state: FSMState = FSMState.FAIL,
    ) -> None:
        """Wait for plan DONE, then hold ``dwell_s`` more before transitioning."""
        if self._waiting_for_plan and self._plan_status == "FAIL":
            self.get_logger().error(f"plan failed in {self._state.name}")
            self._transition(fail_state)
            return
        if not self._plan_done():
            self._done_tick = 0
            if self._waiting_for_plan and self._plan_elapsed_s() >= timeout_s:
                self.get_logger().error(
                    f"{self._state.name} timed out waiting for plan status "
                    f"(last={self._plan_status})"
                )
                self._transition(fail_state)
            return
        self._done_tick += 1
        if self._done_tick * self.TIMER_PERIOD_S >= dwell_s:
            self._transition(nxt)

    def _move(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        duration_s: float = 0.0,
        straight_line: bool = False,
    ) -> None:
        self._begin_plan_wait()
        qx, qy, qz, qw = _yaw_quat(yaw)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "world"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        msg = EETarget()
        msg.pose = pose
        msg.duration_override_s = float(duration_s)
        # Safe transit is intentionally not implemented in the MVP planner path.
        # PRE_GRASP/TRANSIT currently use direct joint-space planning.
        msg.use_safe_transit = False
        msg.straight_line = straight_line
        self._ee_pub.publish(msg)

    def _start_gripper_call(self, close: bool, fail_state: FSMState) -> bool:
        client = self._gripper_close if close else self._gripper_open
        name = "close" if close else "open"
        return self._start_service_call(client, f"gripper/{name}", fail_state)

    def _start_service_call(self, client, name: str, fail_state: FSMState) -> bool:
        if not client.service_is_ready():
            self.get_logger().error(f"{name} service not ready")
            self._transition(fail_state)
            return False
        self._service_future = client.call_async(Trigger.Request())
        self._service_start_s = self._now_s()
        self._service_name = name
        return True

    def _service_succeeded(self) -> bool:
        if self._service_future is None or not self._service_future.done():
            return False
        try:
            res = self._service_future.result()
        except Exception:
            return False
        if not bool(res.success):
            return False
        self._service_future = None
        self._service_name = ""
        return True

    def _service_failed_or_timed_out(self) -> bool:
        if self._service_future is None:
            return False
        if self._service_future.done():
            try:
                res = self._service_future.result()
            except Exception as exc:
                self.get_logger().error(f"{self._service_name} service error: {exc}")
                return True
            if not bool(res.success):
                self.get_logger().error(f"{self._service_name} rejected: {res.message}")
                return True
            return False
        if self._now_s() - self._service_start_s >= self._service_timeout_s:
            self.get_logger().error(f"{self._service_name} service timeout")
            return True
        return False

    def _load_task_presets(self, yaml_path: str) -> dict:
        if not yaml_path:
            return {}
        try:
            with open(yaml_path, "r") as f:
                data = yaml.safe_load(f)
            presets = data.get("task_presets", {})
            self.get_logger().info(f"loaded task presets: {list(presets.keys())} from {yaml_path}")
            return presets
        except Exception as exc:
            self.get_logger().warn(f"failed to load task_presets from '{yaml_path}': {exc}")
            return {}

    def _apply_task_preset(self, task: str) -> None:
        """Apply named task preset; resets to defaults if task is empty or unknown."""
        if task == "default":
            task = ""

        if task:
            p = self._task_presets[task]
            z_pre = float(p.get("z_pregrasp", self._z_pregrasp_default))
            z_gr = float(p.get("z_grasp", self._z_grasp_default))
            z_pl = float(p.get("z_place", self._z_place_default))
            if not (z_pre > z_gr and z_pre > z_pl):
                self.get_logger().error(
                    f"task '{task}': z_pregrasp({z_pre}) must be above z_grasp({z_gr}) "
                    f"and z_place({z_pl}) — using default preset"
                )
                task = ""
            else:
                self._z_pregrasp = z_pre
                self._z_grasp = z_gr
                self._z_place = z_pl
                self._grasp_descend_duration = float(
                    p.get("grasp_descend_duration_s", self._grasp_descend_duration_default)
                )
                self._place_descend_duration = float(
                    p.get("place_descend_duration_s", self._place_descend_duration_default)
                )
                self._post_grasp_hold_s = float(
                    p.get("post_grasp_hold_s", self._post_grasp_hold_default)
                )
                self.get_logger().info(
                    f"preset '{task}': z_pre={self._z_pregrasp} z_gr={self._z_grasp} "
                    f"z_pl={self._z_place} post_grasp_hold={self._post_grasp_hold_s}"
                )

        if not task:
            self._z_pregrasp = self._z_pregrasp_default
            self._z_grasp = self._z_grasp_default
            self._z_place = self._z_place_default
            self._grasp_descend_duration = self._grasp_descend_duration_default
            self._place_descend_duration = self._place_descend_duration_default
            self._post_grasp_hold_s = self._post_grasp_hold_default

    def _begin_plan_wait(self) -> None:
        self._plan_status = "WAITING"
        self._waiting_for_plan = True
        self._plan_started = False
        self._plan_wait_start_s = self._now_s()
        self._done_tick = 0

    def _plan_elapsed_s(self) -> float:
        return self._now_s() - self._plan_wait_start_s

    def _valid_command(self, values: Sequence[float]) -> bool:
        if not all(math.isfinite(v) for v in values):
            self.get_logger().error("pickplace/command contains non-finite value")
            return False
        pick_x, pick_y, _pick_yaw, place_x, place_y, _place_yaw = values
        for label, x, y in (("pick", pick_x, pick_y), ("place", place_x, place_y)):
            if not (self._x_min <= x <= self._x_max and self._y_min <= y <= self._y_max):
                self.get_logger().error(
                    f"{label} target ({x:.3f}, {y:.3f}) outside workspace "
                    f"x=[{self._x_min:.3f}, {self._x_max:.3f}] "
                    f"y=[{self._y_min:.3f}, {self._y_max:.3f}]"
                )
                return False
        return True

    def _validate_startup_params(self) -> bool:
        finite_values = [
            self._z_pregrasp, self._z_grasp, self._z_place,
            self._motion_timeout_s, self._linear_motion_timeout_s,
            self._home_timeout_s, self._service_timeout_s,
            self._x_min, self._x_max, self._y_min, self._y_max,
        ]
        if not all(math.isfinite(v) for v in finite_values):
            self.get_logger().error("task_fsm_node has non-finite parameter")
            return False
        if not (self._z_pregrasp > self._z_grasp and self._z_pregrasp > self._z_place):
            self.get_logger().error(
                "invalid Z params: z_pregrasp must be above z_grasp and z_place"
            )
            return False
        if not (self._x_min < self._x_max and self._y_min < self._y_max):
            self.get_logger().error("invalid workspace bounds")
            return False
        if min(
            self._motion_timeout_s, self._linear_motion_timeout_s,
            self._home_timeout_s, self._service_timeout_s,
        ) <= 0.0:
            self.get_logger().error("timeouts must be positive")
            return False
        return True

    def _publish_fsm_status(self) -> None:
        msg = String()
        msg.data = self._state.name
        self._status_pub.publish(msg)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TaskFSMNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
