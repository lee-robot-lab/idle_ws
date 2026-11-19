# ================================================================
# pick_place_demo.launch.py
# 설명: can_bridge + pick_place_control + demo_supervisor 통합 실기체 데모.
# 사용법:
#   ros2 launch demo_supervisor pick_place_demo.launch.py
#   # 명령: ros2 topic pub --once /demo/command std_msgs/msg/String '{data: "파란 블록을 바구니에"}'
# ================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_WS   = os.path.join(os.path.dirname(__file__), *(['..'] * 6))
_CKPT = os.path.join(_WS, 'checkpoints')


def generate_launch_description():
    return LaunchDescription([
        # ── 파라미터 ─────────────────────────────────────────────────
        DeclareLaunchArgument('stage1_ckpt',    default_value=os.path.join(_CKPT, 'stage1_v2',   'best.pt')),
        DeclareLaunchArgument('color_net_ckpt', default_value=os.path.join(_CKPT, 'color_net_v2', 'best.pt')),
        DeclareLaunchArgument('stage4_ckpt',    default_value=os.path.join(_CKPT, 'stage4',       'best.pt')),
        DeclareLaunchArgument('device',         default_value='cuda'),
        DeclareLaunchArgument('save_dir',       default_value='~/demo_debug'),
        DeclareLaunchArgument('camera_topic',   default_value='/image_raw'),
        DeclareLaunchArgument('parser',         default_value='rule'),
        DeclareLaunchArgument('debug_image',    default_value='false'),

        # ── 1. 로봇 제어 (planner + FSM + gripper) ───────────────────
        # can_bridge_node는 별도 터미널에서: ros2 run can_interface can_bridge_node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('idle_launch'),
                    'launch', 'pick_place_control.launch.py',
                )
            ),
        ),

        # ── 3. 비전/STT supervisor ───────────────────────────────────
        Node(
            package='demo_supervisor',
            executable='demo_supervisor_node',
            name='demo_supervisor',
            output='screen',
            parameters=[{
                'stage1_ckpt':    LaunchConfiguration('stage1_ckpt'),
                'color_net_ckpt': LaunchConfiguration('color_net_ckpt'),
                'stage4_ckpt':    LaunchConfiguration('stage4_ckpt'),
                'device':         LaunchConfiguration('device'),
                'camera_topic':   LaunchConfiguration('camera_topic'),
                'parser':         LaunchConfiguration('parser'),
                'debug_image':    LaunchConfiguration('debug_image'),
                'save_dir':       LaunchConfiguration('save_dir'),
            }],
        ),
    ])
