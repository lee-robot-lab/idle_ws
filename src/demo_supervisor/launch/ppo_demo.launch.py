# ================================================================
# ppo_demo.launch.py
# 설명: PPO direct 실기체 실행. plan_compute + plan_node + gripper + real_action_bridge.
#       task_fsm_node 제외 — PPO가 /ee_target을 직접 publish한다.
# 사용법:
#   ros2 launch demo_supervisor ppo_demo.launch.py
#   # 실제 모터 제어:
#   ros2 run mujoco_phase_rl real_action_bridge \
#       --policy-model <path> --device cuda --armed
# 전제:
#   ros2 run can_interface can_bridge_node  (별도 터미널)
# ================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_WS        = os.path.expanduser('~/idle_ws')
_PPO       = os.path.join(
    _WS, 'src', 'mujoco_phase_rl', 'outputs',
    'ppo_recovery_fixed_s0', 'checkpoints',
    'ppo_recovery_fixed_s0_98304_steps.zip',
)
_CKPT      = os.path.join(_WS, 'checkpoints')
_STAGE1    = os.path.join(_CKPT, 'stage1_v2', 'best.pt')
_COLOR_NET = os.path.join(_CKPT, 'color_net_v2', 'best.pt')
_SLOT_DIFF = os.path.join(_CKPT, 'slot_diff', 'best.pt')
_STAGE4    = os.path.join(_CKPT, 'stage4', 'best.pt')


def generate_launch_description():
    return LaunchDescription([
        # ── planner 파라미터 (pick_place_control.launch.py 기본값 그대로) ──
        DeclareLaunchArgument('planner_v_max',                   default_value='1.0'),
        DeclareLaunchArgument('planner_a_max',                   default_value='1.0'),
        DeclareLaunchArgument('j1_traj_fraction',                default_value='1.0'),
        DeclareLaunchArgument('floor_collision',                  default_value='false'),
        DeclareLaunchArgument('cage_collision',                   default_value='true'),
        DeclareLaunchArgument('kp_max',                          default_value='60.0'),
        DeclareLaunchArgument('kd_max',                          default_value='10.0'),
        DeclareLaunchArgument('settle_kp_scale_by_motor_json',   default_value='{"1": 1.6, "2": 1.6, "4": 1}'),
        DeclareLaunchArgument('settle_kd_scale_by_motor_json',   default_value='{"1": 1, "2": 1.2}'),
        DeclareLaunchArgument('settle_gain_ramp_s',              default_value='0.4'),
        DeclareLaunchArgument('settle_blend_before_end_s',       default_value='1.5'),
        DeclareLaunchArgument('settle_velocity_brake_kd_scale',  default_value='2.0'),
        DeclareLaunchArgument('settle_velocity_brake_full_vel_rad_s', default_value='0.10'),
        DeclareLaunchArgument('settle_friction_scale',           default_value='0.5'),
        DeclareLaunchArgument('hold_friction_scale',             default_value='0.0'),
        DeclareLaunchArgument('hold_friction_scale_by_motor_json', default_value='{"2": 0}'),
        DeclareLaunchArgument('hold_friction_deadband_rad',      default_value='0.00'),
        DeclareLaunchArgument('hold_kp_scale_by_motor_json',     default_value='{"1": 0, "2": 0, "3": 0, "4": 0, "5": 0, "6": 0}'),
        DeclareLaunchArgument('hold_kd_scale_by_motor_json',     default_value='{"1": 0.0, "2": 0.0, "3": 0.0, "4": 0.0, "5": 0.0, "6": 0.0}'),
        DeclareLaunchArgument('hold_latch_actual_q_after_settle', default_value='true'),
        DeclareLaunchArgument('hold_latch_max_err_rad',          default_value='0.008'),
        DeclareLaunchArgument('settle_vel_rad_s',                default_value='0.12'),
        DeclareLaunchArgument('settle_kd_scale',                 default_value='0.7'),
        DeclareLaunchArgument('hold_qd_lpf_alpha',               default_value='0.95'),
        DeclareLaunchArgument('settle_qd_lpf_alpha',             default_value='0.7'),
        DeclareLaunchArgument('plan_diag_csv_path',              default_value=''),
        DeclareLaunchArgument('plan_diag_hz',                    default_value='100.0'),

        # ── PPO 브릿지 파라미터 ────────────────────────────────────────
        DeclareLaunchArgument('policy_model',       default_value=_PPO),
        DeclareLaunchArgument('device',             default_value='cuda'),
        DeclareLaunchArgument('camera_topic',       default_value='/image_raw'),
        DeclareLaunchArgument('target_color',       default_value=''),
        DeclareLaunchArgument('task_mode',          default_value='basket'),
        DeclareLaunchArgument('phase_prior_weight', default_value='0.8'),
        DeclareLaunchArgument('slot_stage1_ckpt',   default_value=_STAGE1),
        DeclareLaunchArgument('slot_diff_ckpt',     default_value=_SLOT_DIFF),
        DeclareLaunchArgument('slot_color_net_ckpt', default_value=_COLOR_NET),
        DeclareLaunchArgument('stage4_ckpt',        default_value=_STAGE4),
        DeclareLaunchArgument('parser',             default_value='qwen'),
        DeclareLaunchArgument('armed',              default_value='false'),
        DeclareLaunchArgument('image_device',       default_value='1',
                              description='cv2.VideoCapture index (-1 = use /image_raw topic)'),
        DeclareLaunchArgument('whisper_model_size', default_value='',
                              description='faster_whisper size (e.g. small). 빈값=topic 사용'),

        # ── 1. IK 계산 ────────────────────────────────────────────────
        Node(
            package='phy',
            executable='plan_compute_node',
            name='plan_compute_node',
            output='screen',
            parameters=[{
                'planner_v_max':    ParameterValue(LaunchConfiguration('planner_v_max'), value_type=float),
                'planner_a_max':    ParameterValue(LaunchConfiguration('planner_a_max'), value_type=float),
                'floor_collision':  LaunchConfiguration('floor_collision'),
                'cage_collision':   LaunchConfiguration('cage_collision'),
            }],
        ),

        # ── 2. trajectory 실행 ────────────────────────────────────────
        Node(
            package='phy',
            executable='plan_node',
            name='plan_node',
            output='screen',
            parameters=[{
                'planner_v_max':    ParameterValue(LaunchConfiguration('planner_v_max'), value_type=float),
                'planner_a_max':    ParameterValue(LaunchConfiguration('planner_a_max'), value_type=float),
                'j1_traj_fraction': ParameterValue(LaunchConfiguration('j1_traj_fraction'), value_type=float),
                'kp_max':           ParameterValue(LaunchConfiguration('kp_max'), value_type=float),
                'kd_max':           ParameterValue(LaunchConfiguration('kd_max'), value_type=float),
                'settle_kp_scale_by_motor_json':          ParameterValue(LaunchConfiguration('settle_kp_scale_by_motor_json'), value_type=str),
                'settle_kd_scale_by_motor_json':          ParameterValue(LaunchConfiguration('settle_kd_scale_by_motor_json'), value_type=str),
                'settle_gain_ramp_s':                     ParameterValue(LaunchConfiguration('settle_gain_ramp_s'), value_type=float),
                'settle_blend_before_end_s':              ParameterValue(LaunchConfiguration('settle_blend_before_end_s'), value_type=float),
                'settle_velocity_brake_kd_scale':         ParameterValue(LaunchConfiguration('settle_velocity_brake_kd_scale'), value_type=float),
                'settle_velocity_brake_full_vel_rad_s':   ParameterValue(LaunchConfiguration('settle_velocity_brake_full_vel_rad_s'), value_type=float),
                'settle_friction_scale':                  ParameterValue(LaunchConfiguration('settle_friction_scale'), value_type=float),
                'hold_friction_scale':                    ParameterValue(LaunchConfiguration('hold_friction_scale'), value_type=float),
                'hold_friction_scale_by_motor_json':      ParameterValue(LaunchConfiguration('hold_friction_scale_by_motor_json'), value_type=str),
                'hold_friction_deadband_rad':             ParameterValue(LaunchConfiguration('hold_friction_deadband_rad'), value_type=float),
                'hold_kp_scale_by_motor_json':            ParameterValue(LaunchConfiguration('hold_kp_scale_by_motor_json'), value_type=str),
                'hold_kd_scale_by_motor_json':            ParameterValue(LaunchConfiguration('hold_kd_scale_by_motor_json'), value_type=str),
                'hold_latch_actual_q_after_settle':       LaunchConfiguration('hold_latch_actual_q_after_settle'),
                'hold_latch_max_err_rad':                 ParameterValue(LaunchConfiguration('hold_latch_max_err_rad'), value_type=float),
                'settle_vel_rad_s':                       ParameterValue(LaunchConfiguration('settle_vel_rad_s'), value_type=float),
                'settle_kd_scale':                        ParameterValue(LaunchConfiguration('settle_kd_scale'), value_type=float),
                'hold_qd_lpf_alpha':                      ParameterValue(LaunchConfiguration('hold_qd_lpf_alpha'), value_type=float),
                'settle_qd_lpf_alpha':                    ParameterValue(LaunchConfiguration('settle_qd_lpf_alpha'), value_type=float),
                'plan_diag_csv_path':                     LaunchConfiguration('plan_diag_csv_path'),
                'plan_diag_hz':                           ParameterValue(LaunchConfiguration('plan_diag_hz'), value_type=float),
            }],
        ),

        # ── 3. 그리퍼 ─────────────────────────────────────────────────
        Node(
            package='phy',
            executable='gripper_node',
            name='gripper_node',
            output='screen',
        ),

        # ── 4. PPO real_action_bridge (dry-run 기본) ──────────────────
        # emulate_tty=True: stdin을 TTY로 유지 → spacebar STT 동작
        Node(
            package='mujoco_phase_rl',
            executable='real_action_bridge',
            name='ppo_action_bridge',
            output='screen',
            emulate_tty=True,
            arguments=[
                '--policy-model',        LaunchConfiguration('policy_model'),
                '--device',              LaunchConfiguration('device'),
                '--image-topic',         LaunchConfiguration('camera_topic'),
                '--target-color',        LaunchConfiguration('target_color'),
                '--task-mode',           LaunchConfiguration('task_mode'),
                '--phase-prior-weight',  LaunchConfiguration('phase_prior_weight'),
                '--slot-stage1-ckpt',    LaunchConfiguration('slot_stage1_ckpt'),
                '--slot-diff-ckpt',      LaunchConfiguration('slot_diff_ckpt'),
                '--slot-color-net-ckpt', LaunchConfiguration('slot_color_net_ckpt'),
                '--stage4-ckpt',         LaunchConfiguration('stage4_ckpt'),
                '--image-device',        LaunchConfiguration('image_device'),
                '--whisper-model-size',  LaunchConfiguration('whisper_model_size'),
            ],
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('armed'), "' != 'true'"])),
        ),

        # ── 4b. PPO real_action_bridge (ARMED) ───────────────────────
        Node(
            package='mujoco_phase_rl',
            executable='real_action_bridge',
            name='ppo_action_bridge',
            output='screen',
            emulate_tty=True,
            arguments=[
                '--policy-model',        LaunchConfiguration('policy_model'),
                '--device',              LaunchConfiguration('device'),
                '--image-topic',         LaunchConfiguration('camera_topic'),
                '--target-color',        LaunchConfiguration('target_color'),
                '--task-mode',           LaunchConfiguration('task_mode'),
                '--phase-prior-weight',  LaunchConfiguration('phase_prior_weight'),
                '--slot-stage1-ckpt',    LaunchConfiguration('slot_stage1_ckpt'),
                '--slot-diff-ckpt',      LaunchConfiguration('slot_diff_ckpt'),
                '--slot-color-net-ckpt', LaunchConfiguration('slot_color_net_ckpt'),
                '--stage4-ckpt',         LaunchConfiguration('stage4_ckpt'),
                '--image-device',        LaunchConfiguration('image_device'),
                '--whisper-model-size',  LaunchConfiguration('whisper_model_size'),
                '--armed',
            ],
            condition=IfCondition(LaunchConfiguration('armed')),
        ),

        # ── 5. STT+Grounding supervisor — whisper_model_size 비어있을 때만 기동 ──
        # whisper 지정 시 real_action_bridge가 내부에서 직접 처리하므로 불필요
        Node(
            package='demo_supervisor',
            executable='demo_supervisor_node',
            name='demo_supervisor_node',
            output='screen',
            parameters=[{
                'stage1_ckpt':     LaunchConfiguration('slot_stage1_ckpt'),
                'color_net_ckpt':  LaunchConfiguration('slot_color_net_ckpt'),
                'stage4_ckpt':     LaunchConfiguration('stage4_ckpt'),
                'device':          LaunchConfiguration('device'),
                'parser':          LaunchConfiguration('parser'),
                'ppo_mode':        True,
                'camera_topic':    LaunchConfiguration('camera_topic'),
            }],
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('whisper_model_size'), "' == ''"])),
        ),
    ])
