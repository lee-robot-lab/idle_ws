"""Launch sim_driver (physics + viewer) + plan_node + task_fsm_node + gripper_node.

Pick-and-place 데모:
    ros2 topic pub --once /pickplace/command msgs/msg/PickPlaceCommand \\
        '{task: "", x_pick: 0.18, y_pick: 0.30, yaw_pick: 0.0,
                   x_place: 0.0, y_place: 0.62, yaw_place: 0.0}'
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    viewer_arg = DeclareLaunchArgument(
        "viewer",
        default_value="true",
        description="Enable MuJoCo viewer window (physics-coupled, mouse perturbation enabled)",
    )
    viewer_left_ui_arg = DeclareLaunchArgument(
        "viewer_left_ui",
        default_value="true",
        description="Show MuJoCo left UI panel",
    )
    viewer_right_ui_arg = DeclareLaunchArgument(
        "viewer_right_ui",
        default_value="true",
        description="Show MuJoCo right UI panel",
    )
    v_max_arg = DeclareLaunchArgument(
        "planner_v_max",
        default_value="1.0",
        description="Per-joint max velocity for trajectory generation (rad/s)",
    )
    a_max_arg = DeclareLaunchArgument(
        "planner_a_max",
        default_value="1.0",
        description="Per-joint max acceleration for trajectory generation (rad/s²)",
    )
    disable_gravity_arg = DeclareLaunchArgument(
        "disable_gravity",
        default_value="false",
        description="Disable gravity comp in plan_node (only for sim with dummy inertials)",
    )
    unlimited_tau_arg = DeclareLaunchArgument(
        "unlimited_tau",
        default_value="false",
        description="Bypass tau_ff clipping in plan_node (sim with dummy inertials)",
    )
    warp_q_lo_arg = DeclareLaunchArgument(
        "warp_q_lo_rad",
        default_value="0.12",
        description="Tracking error below this keeps trajectory time warp at 1.0",
    )
    warp_q_hi_arg = DeclareLaunchArgument(
        "warp_q_hi_rad",
        default_value="0.40",
        description="Tracking error above this stops trajectory virtual time",
    )
    j1_traj_fraction_arg = DeclareLaunchArgument(
        "j1_traj_fraction",
        default_value="1.0",
        description="j1 virtual time fraction (<1.0): j1 reaches goal early, excluded from warp",
    )
    disable_scene_contacts_arg = DeclareLaunchArgument(
        "disable_scene_contacts",
        default_value="false",
        description="Disable basket/block MuJoCo contacts for reachability sweeps",
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("IDLE_PARAM_ROOT", "/home/su/idle_ws/param/sim"),
            viewer_arg,
            viewer_left_ui_arg,
            viewer_right_ui_arg,
            v_max_arg,
            a_max_arg,
            disable_gravity_arg,
            unlimited_tau_arg,
            warp_q_lo_arg,
            warp_q_hi_arg,
            j1_traj_fraction_arg,
            disable_scene_contacts_arg,
            Node(
                package="sim",
                executable="sim_driver_node",
                name="sim_driver_node",
                output="screen",
                parameters=[
                    {
                        "viewer": LaunchConfiguration("viewer"),
                        "viewer_left_ui": LaunchConfiguration("viewer_left_ui"),
                        "viewer_right_ui": LaunchConfiguration("viewer_right_ui"),
                        "disable_scene_contacts": LaunchConfiguration("disable_scene_contacts"),
                    }
                ],
            ),
            Node(
                package="phy",
                executable="plan_compute_node",
                name="plan_compute_node",
                output="screen",
                parameters=[
                    {
                        "planner_v_max": LaunchConfiguration("planner_v_max"),
                        "planner_a_max": LaunchConfiguration("planner_a_max"),
                    }
                ],
            ),
            Node(
                package="phy",
                executable="plan_node",
                name="plan_node",
                output="screen",
                parameters=[
                    {
                        "planner_v_max": LaunchConfiguration("planner_v_max"),
                        "planner_a_max": LaunchConfiguration("planner_a_max"),
                        "disable_gravity": LaunchConfiguration("disable_gravity"),
                        "unlimited_tau": LaunchConfiguration("unlimited_tau"),
                        "warp_q_lo_rad": LaunchConfiguration("warp_q_lo_rad"),
                        "warp_q_hi_rad": LaunchConfiguration("warp_q_hi_rad"),
                        "j1_traj_fraction": LaunchConfiguration("j1_traj_fraction"),
                        "settle_timeout_s": 5.0,
                        "kp_max": 60.0,
                    }
                ],
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
                parameters=[{
                    "z_pregrasp": 0.40,
                    "z_grasp": 0.12,
                    "z_place": 0.25,
                    "x_min": -0.5,
                    "x_max": 0.5,
                    "y_min": -0.1,
                    "y_max": 0.9,
                    "task_presets_yaml_path": "/home/su/idle_ws/param/tuned/task_presets.yaml",
                }],
            ),
        ]
    )
