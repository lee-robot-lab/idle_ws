import sys
from types import ModuleType, SimpleNamespace


def _install_ros_stubs():
    rclpy = ModuleType("rclpy")
    rclpy.init = lambda *args, **kwargs: None
    rclpy.spin = lambda *args, **kwargs: None
    rclpy.ok = lambda: False
    rclpy.shutdown = lambda: None
    sys.modules.setdefault("rclpy", rclpy)

    executors = ModuleType("rclpy.executors")
    executors.ExternalShutdownException = RuntimeError
    sys.modules.setdefault("rclpy.executors", executors)

    node_mod = ModuleType("rclpy.node")

    class Node:
        pass

    node_mod.Node = Node
    sys.modules.setdefault("rclpy.node", node_mod)

    geometry_msgs = ModuleType("geometry_msgs")
    geometry_msgs_msg = ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.PoseStamped = object
    sys.modules.setdefault("geometry_msgs", geometry_msgs)
    sys.modules.setdefault("geometry_msgs.msg", geometry_msgs_msg)

    msgs = ModuleType("msgs")
    msgs_msg = ModuleType("msgs.msg")
    msgs_msg.EETarget = object
    msgs_msg.PickPlaceCommand = object
    sys.modules.setdefault("msgs", msgs)
    sys.modules.setdefault("msgs.msg", msgs_msg)

    std_msgs = ModuleType("std_msgs")
    std_msgs_msg = ModuleType("std_msgs.msg")
    std_msgs_msg.Bool = object

    class String:
        def __init__(self):
            self.data = ""

    std_msgs_msg.String = String
    sys.modules.setdefault("std_msgs", std_msgs)
    sys.modules.setdefault("std_msgs.msg", std_msgs_msg)

    std_srvs = ModuleType("std_srvs")
    std_srvs_srv = ModuleType("std_srvs.srv")

    class Trigger:
        class Request:
            pass

    std_srvs_srv.Trigger = Trigger
    sys.modules.setdefault("std_srvs", std_srvs)
    sys.modules.setdefault("std_srvs.srv", std_srvs_srv)


_install_ros_stubs()

from phy.task_fsm_node import FSMState, TaskFSMNode


class _Logger:
    def info(self, _msg):
        pass

    def warn(self, _msg):
        pass

    def error(self, _msg):
        pass


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _Future:
    def __init__(self, *, done=False, success=True):
        self._done = done
        self._success = success

    def done(self):
        return self._done

    def result(self):
        return SimpleNamespace(success=self._success, message="")


class _Client:
    def __init__(self, future):
        self.future = future
        self.calls = 0

    def service_is_ready(self):
        return True

    def call_async(self, _request):
        self.calls += 1
        return self.future


def _fault_node(future):
    node = TaskFSMNode.__new__(TaskFSMNode)
    node._state = FSMState.FAULT
    node._state_ticks = 0
    node._done_tick = 0
    node._plan_status = "IDLE"
    node._waiting_for_plan = False
    node._plan_started = False
    node._plan_wait_start_s = 0.0
    node._service_future = None
    node._service_start_s = 0.0
    node._service_name = ""
    node._home_accepted = False
    node._gripper_close_accepted = False
    node._gripper_open_accepted = False
    node._fault_home_attempted = False
    node._go_home = _Client(future)
    node._home_timeout_s = 15.0
    node._dwell_home_s = 0.0
    node._service_timeout_s = 2.0
    node.TIMER_PERIOD_S = 0.1
    node._status_pub = _Publisher()
    node.get_logger = lambda: _Logger()
    node._now_s = lambda: 10.0
    return node


def test_fault_starts_single_home_recovery_attempt():
    node = _fault_node(_Future(done=False))

    node._on_timer()
    node._on_timer()

    assert node._go_home.calls == 1
    assert node._state is FSMState.FAULT
    assert node._home_accepted is False


def test_fault_home_recovery_returns_to_idle_after_plan_done():
    node = _fault_node(_Future(done=True, success=True))

    node._on_timer()
    node._on_timer()
    node._plan_status = "DONE"
    node._on_timer()

    assert node._state is FSMState.IDLE


def test_fault_home_recovery_stays_faulted_when_service_fails():
    node = _fault_node(_Future(done=True, success=False))

    node._on_timer()
    node._on_timer()

    assert node._state is FSMState.FAULT
    assert node._go_home.calls == 1
