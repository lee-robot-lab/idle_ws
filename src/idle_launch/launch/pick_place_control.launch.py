"""pick_place_control — plan_compute_node + plan_node + gripper_node + task_fsm_node.

실 하드웨어용 (sim 없음). can_bridge_node는 별도 터미널에서 실행.

사용 예:
    ros2 launch idle_launch pick_place_control.launch.py
    ros2 topic pub --once /pickplace/command msgs/msg/PickPlaceCommand \
        '{task: "stack", x_pick: 0.30, y_pick: 0.0, yaw_pick: 0.0, x_place: 0.30, y_place: 0.20, yaw_place: 0.0}'
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    _default_presets = "/home/su/idle_ws/param/tuned/task_presets.yaml"

    j1_traj_fraction_arg = DeclareLaunchArgument(
        "j1_traj_fraction",
        default_value="1.0",
        description="j1 virtual time fraction (<1.0): j1 reaches goal early, excluded from warp",
    )
    v_max_arg = DeclareLaunchArgument(
        "planner_v_max",
        default_value="1.0",
        description="Per-joint max velocity (rad/s)",
    )
    a_max_arg = DeclareLaunchArgument(
        "planner_a_max",
        default_value="1.0",
        description="Per-joint max acceleration (rad/s²)",
    )
    floor_collision_arg = DeclareLaunchArgument(
        "floor_collision",
        default_value="false",
        description="Enable floor collision checking (finger_r/l/gripper vs floor)",
    )
    cage_collision_arg = DeclareLaunchArgument(
        "cage_collision",
        default_value="true",
        description="Enable cage collision checking (arm vs cage mesh)",
    )
    settle_kp_scale_by_motor_arg = DeclareLaunchArgument(
        "settle_kp_scale_by_motor_json",
        default_value='{1: 1, 2: 1.4, 4: 1}',
        description="Per-motor kp scale used only while settling to q_final",
    )
    settle_kd_scale_by_motor_arg = DeclareLaunchArgument(
        "settle_kd_scale_by_motor_json",
        default_value='{1: 1, 2: 1.2}',
        description="Per-motor kd scale used only while settling to q_final",
    )
    settle_gain_ramp_arg = DeclareLaunchArgument(
        "settle_gain_ramp_s",
        default_value="0.4",
        description="Ramp duration for settle gain scale to avoid torque steps",
    )
    settle_blend_before_end_arg = DeclareLaunchArgument(
        "settle_blend_before_end_s",
        default_value="1.5",
        description="Blend settle gain/friction into the final trajectory segment",
    )
    settle_velocity_brake_kd_scale_arg = DeclareLaunchArgument(
        "settle_velocity_brake_kd_scale",
        default_value="2.0",
        description="Extra settle kd scale near q_final while joint velocity is above target",
    )
    settle_velocity_brake_full_vel_arg = DeclareLaunchArgument(
        "settle_velocity_brake_full_vel_rad_s",
        default_value="0.10",
        description="Joint velocity where settle velocity brake reaches full kd scale",
    )
    settle_friction_scale_arg = DeclareLaunchArgument(
        "settle_friction_scale",
        default_value="1.0",
        description="Friction feedforward scale while settling to q_final",
    )
    hold_friction_scale_arg = DeclareLaunchArgument(
        "hold_friction_scale",
        default_value="0.0",
        description="Friction feedforward scale after DONE hold (global)",
    )
    hold_friction_scale_by_motor_arg = DeclareLaunchArgument(
        "hold_friction_scale_by_motor_json",
        default_value='{"2": 0.3}',
        description="Per-motor friction scale in hold (overrides global). j2=0.3으로 static error 보정",
    )
    hold_friction_deadband_arg = DeclareLaunchArgument(
        "hold_friction_deadband_rad",
        default_value="0.00",
        description="Error deadband below which friction FF is suppressed in hold",
    )
    hold_kp_scale_by_motor_arg = DeclareLaunchArgument(
        "hold_kp_scale_by_motor_json",
        default_value="{ 1: 1, 2: 1.8, 4: 1 }",
        description="Per-motor kp scale used after DONE hold",
    )
    hold_kd_scale_by_motor_arg = DeclareLaunchArgument(
        "hold_kd_scale_by_motor_json",
        default_value='{}',
        description="Per-motor kd scale used after DONE hold",
    )
    hold_latch_actual_q_arg = DeclareLaunchArgument(
        "hold_latch_actual_q_after_settle",
        default_value="true",
        description="Latch actual joint positions as hold setpoints after near-final settle",
    )
    hold_latch_max_err_arg = DeclareLaunchArgument(
        "hold_latch_max_err_rad",
        default_value="0.008",
        description="Max q_final tracking error that allows actual-q hold latch",
    )
    kp_max_arg = DeclareLaunchArgument(
        "kp_max",
        default_value="60.0",
        description="Software clamp for outgoing motor kp",
    )
    kd_max_arg = DeclareLaunchArgument(
        "kd_max",
        default_value="10.0",
        description="Software clamp for outgoing motor kd",
    )
    settle_vel_rad_s_arg = DeclareLaunchArgument(
        "settle_vel_rad_s",
        default_value="0.12",
        description="Max joint velocity to declare settle done (raised from 0.05 — noise floor is 0.07~0.09)",
    )
    settle_kd_scale_arg = DeclareLaunchArgument(
        "settle_kd_scale",
        default_value="0.7",
        description="Global kd scale while settling (reduces qd-noise torque)",
    )
    hold_qd_lpf_alpha_arg = DeclareLaunchArgument(
        "hold_qd_lpf_alpha",
        default_value="0.95",
        description="IIR alpha for hold-phase qd low-pass filter (method B)",
    )
    settle_qd_lpf_alpha_arg = DeclareLaunchArgument(
        "settle_qd_lpf_alpha",
        default_value="0.9",
        description="IIR alpha for settle-phase qd low-pass filter (method B)",
    )
    plan_diag_csv_path_arg = DeclareLaunchArgument(
        "plan_diag_csv_path",
        default_value="",
        description="Optional plan_node joint diagnostic CSV path",
    )
    plan_diag_hz_arg = DeclareLaunchArgument(
        "plan_diag_hz",
        default_value="100.0",
        description="plan_node joint diagnostic CSV sample rate",
    )

    v_max = ParameterValue(LaunchConfiguration("planner_v_max"), value_type=float)
    a_max = ParameterValue(LaunchConfiguration("planner_a_max"), value_type=float)
    floor_collision = LaunchConfiguration("floor_collision")
    cage_collision = LaunchConfiguration("cage_collision")

    return LaunchDescription([
        j1_traj_fraction_arg,
        v_max_arg,
        a_max_arg,
        floor_collision_arg,
        cage_collision_arg,
        settle_kp_scale_by_motor_arg,
        settle_kd_scale_by_motor_arg,
        settle_gain_ramp_arg,
        settle_blend_before_end_arg,
        settle_velocity_brake_kd_scale_arg,
        settle_velocity_brake_full_vel_arg,
        settle_friction_scale_arg,
        hold_friction_scale_arg,
        hold_friction_scale_by_motor_arg,
        hold_friction_deadband_arg,
        hold_kp_scale_by_motor_arg,
        hold_kd_scale_by_motor_arg,
        hold_latch_actual_q_arg,
        hold_latch_max_err_arg,
        kp_max_arg,
        kd_max_arg,
        settle_vel_rad_s_arg,
        settle_kd_scale_arg,
        hold_qd_lpf_alpha_arg,
        settle_qd_lpf_alpha_arg,
        plan_diag_csv_path_arg,
        plan_diag_hz_arg,
        Node(
            package="phy",
            executable="plan_compute_node",
            name="plan_compute_node",
            output="screen",
            parameters=[{
                "planner_v_max": v_max,
                "planner_a_max": a_max,
                "floor_collision": floor_collision,
                "cage_collision": cage_collision,
            }],
        ),
        Node(
            package="phy",
            executable="plan_node",
            name="plan_node",
            output="screen",
            parameters=[{
                "planner_v_max": v_max,
                "planner_a_max": a_max,
                "j1_traj_fraction": ParameterValue(
                    LaunchConfiguration("j1_traj_fraction"),
                    value_type=float,
                ),
                "kp_max": ParameterValue(
                    LaunchConfiguration("kp_max"),
                    value_type=float,
                ),
                "kd_max": ParameterValue(
                    LaunchConfiguration("kd_max"),
                    value_type=float,
                ),
                "settle_kp_scale_by_motor_json": ParameterValue(
                    LaunchConfiguration("settle_kp_scale_by_motor_json"),
                    value_type=str,
                ),
                "settle_kd_scale_by_motor_json": ParameterValue(
                    LaunchConfiguration("settle_kd_scale_by_motor_json"),
                    value_type=str,
                ),
                "settle_gain_ramp_s": ParameterValue(
                    LaunchConfiguration("settle_gain_ramp_s"),
                    value_type=float,
                ),
                "settle_blend_before_end_s": ParameterValue(
                    LaunchConfiguration("settle_blend_before_end_s"),
                    value_type=float,
                ),
                "settle_velocity_brake_kd_scale": ParameterValue(
                    LaunchConfiguration("settle_velocity_brake_kd_scale"),
                    value_type=float,
                ),
                "settle_velocity_brake_full_vel_rad_s": ParameterValue(
                    LaunchConfiguration("settle_velocity_brake_full_vel_rad_s"),
                    value_type=float,
                ),
                "settle_friction_scale": ParameterValue(
                    LaunchConfiguration("settle_friction_scale"),
                    value_type=float,
                ),
                "hold_friction_scale": ParameterValue(
                    LaunchConfiguration("hold_friction_scale"),
                    value_type=float,
                ),
                "hold_friction_scale_by_motor_json": ParameterValue(
                    LaunchConfiguration("hold_friction_scale_by_motor_json"),
                    value_type=str,
                ),
                "hold_friction_deadband_rad": ParameterValue(
                    LaunchConfiguration("hold_friction_deadband_rad"),
                    value_type=float,
                ),
                "hold_kp_scale_by_motor_json": ParameterValue(
                    LaunchConfiguration("hold_kp_scale_by_motor_json"),
                    value_type=str,
                ),
                "hold_kd_scale_by_motor_json": ParameterValue(
                    LaunchConfiguration("hold_kd_scale_by_motor_json"),
                    value_type=str,
                ),
                "hold_latch_actual_q_after_settle": LaunchConfiguration("hold_latch_actual_q_after_settle"),
                "hold_latch_max_err_rad": ParameterValue(
                    LaunchConfiguration("hold_latch_max_err_rad"),
                    value_type=float,
                ),
                "settle_vel_rad_s": ParameterValue(
                    LaunchConfiguration("settle_vel_rad_s"),
                    value_type=float,
                ),
                "settle_kd_scale": ParameterValue(
                    LaunchConfiguration("settle_kd_scale"),
                    value_type=float,
                ),
                "hold_qd_lpf_alpha": ParameterValue(
                    LaunchConfiguration("hold_qd_lpf_alpha"),
                    value_type=float,
                ),
                "settle_qd_lpf_alpha": ParameterValue(
                    LaunchConfiguration("settle_qd_lpf_alpha"),
                    value_type=float,
                ),
                "plan_diag_csv_path": LaunchConfiguration("plan_diag_csv_path"),
                "plan_diag_hz": ParameterValue(
                    LaunchConfiguration("plan_diag_hz"),
                    value_type=float,
                ),
            }],
        ),
        Node(
            package="phy",
            executable="gripper_node",
            name="gripper_node",
            output="screen",
        ),
        Node(
            package="phy",
            executable="task_fsm_node",
            name="task_fsm_node",
            output="screen",
            parameters=[{"task_presets_yaml_path": _default_presets}],
        ),
    ])
