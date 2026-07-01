import ast
from pathlib import Path


LAUNCH_PATH = Path(__file__).resolve().parents[1] / "launch" / "pick_place_control.launch.py"


def _launch_source() -> str:
    return LAUNCH_PATH.read_text(encoding="utf-8")


def test_pick_place_control_exposes_hold_kd_scale_map():
    source = _launch_source()
    tree = ast.parse(source)

    declared_names = {
        node.args[0].value
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and getattr(node.func, "id", "") == "DeclareLaunchArgument"
        and node.args
        and isinstance(node.args[0], ast.Constant)
    }

    assert "hold_kd_scale_by_motor_json" in declared_names
    assert '"hold_kd_scale_by_motor_json"' in source
    assert "LaunchConfiguration(\"hold_kd_scale_by_motor_json\")" in source
    assert "value_type=str" in source


def test_pick_place_control_exposes_hold_kp_scale_map():
    source = _launch_source()
    tree = ast.parse(source)

    declared_names = {
        node.args[0].value
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and getattr(node.func, "id", "") == "DeclareLaunchArgument"
        and node.args
        and isinstance(node.args[0], ast.Constant)
    }

    assert "hold_kp_scale_by_motor_json" in declared_names
    assert '"hold_kp_scale_by_motor_json"' in source
    assert "LaunchConfiguration(\"hold_kp_scale_by_motor_json\")" in source
    assert "value_type=str" in source


def test_pick_place_control_exposes_hold_latch_controls():
    source = _launch_source()
    tree = ast.parse(source)

    declared_names = {
        node.args[0].value
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and getattr(node.func, "id", "") == "DeclareLaunchArgument"
        and node.args
        and isinstance(node.args[0], ast.Constant)
    }

    assert "hold_latch_actual_q_after_settle" in declared_names
    assert "hold_latch_max_err_rad" in declared_names
    assert "LaunchConfiguration(\"hold_latch_actual_q_after_settle\")" in source
    assert "LaunchConfiguration(\"hold_latch_max_err_rad\")" in source


def test_pick_place_control_exposes_settle_velocity_brake_controls():
    source = _launch_source()
    tree = ast.parse(source)

    declared_names = {
        node.args[0].value
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and getattr(node.func, "id", "") == "DeclareLaunchArgument"
        and node.args
        and isinstance(node.args[0], ast.Constant)
    }

    assert "settle_velocity_brake_kd_scale" in declared_names
    assert "settle_velocity_brake_full_vel_rad_s" in declared_names
    assert "LaunchConfiguration(\"settle_velocity_brake_kd_scale\")" in source
    assert "LaunchConfiguration(\"settle_velocity_brake_full_vel_rad_s\")" in source


def test_pick_place_control_exposes_task_presets_yaml_path():
    source = _launch_source()
    tree = ast.parse(source)

    declared_names = {
        node.args[0].value
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and getattr(node.func, "id", "") == "DeclareLaunchArgument"
        and node.args
        and isinstance(node.args[0], ast.Constant)
    }

    assert "task_presets_yaml_path" in declared_names
    assert "LaunchConfiguration(\"task_presets_yaml_path\")" in source


def test_pick_place_control_exposes_grasp_dwell():
    source = _launch_source()
    tree = ast.parse(source)

    declared_names = {
        node.args[0].value
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and getattr(node.func, "id", "") == "DeclareLaunchArgument"
        and node.args
        and isinstance(node.args[0], ast.Constant)
    }

    assert "dwell_grasp_s" in declared_names
    assert '"dwell_grasp_s"' in source
    assert "LaunchConfiguration(\"dwell_grasp_s\")" in source


def test_pick_place_control_exposes_gripper_grasp_settle_ticks():
    source = _launch_source()
    tree = ast.parse(source)

    declared_names = {
        node.args[0].value
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and getattr(node.func, "id", "") == "DeclareLaunchArgument"
        and node.args
        and isinstance(node.args[0], ast.Constant)
    }

    assert "gripper_grasp_settle_ticks" in declared_names
    assert '"grasp_settle_ticks"' in source
    assert "LaunchConfiguration(\"gripper_grasp_settle_ticks\")" in source
