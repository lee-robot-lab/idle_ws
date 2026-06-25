"""Launch a USB RGB camera with RGB-only box pose detection."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DEFAULT_VIDEO_DEVICE = os.getenv("IDLE_VISION_VIDEO_DEVICE", "/dev/video2")
DEFAULT_PLANE_HOMOGRAPHY_JSON = os.getenv(
    "IDLE_VISION_PLANE_HOMOGRAPHY_JSON",
    "[[0.0009504612,-2.1327e-06,-0.5866006127],"
    "[1.9451e-06,-0.0009616124,0.928124009],"
    "[-6.2509e-06,-2.12835e-05,1.0]]",
)


def generate_launch_description() -> LaunchDescription:
    start_camera_arg = DeclareLaunchArgument("start_camera", default_value="true")
    start_rqt_arg = DeclareLaunchArgument("start_rqt", default_value="true")
    video_device_arg = DeclareLaunchArgument(
        "video_device",
        default_value=DEFAULT_VIDEO_DEVICE,
    )
    image_width_arg = DeclareLaunchArgument("image_width", default_value="1280")
    image_height_arg = DeclareLaunchArgument("image_height", default_value="720")
    framerate_arg = DeclareLaunchArgument("framerate", default_value="30.0")
    pixel_format_arg = DeclareLaunchArgument("pixel_format", default_value="mjpeg2rgb")
    camera_frame_id_arg = DeclareLaunchArgument(
        "camera_frame_id",
        default_value="usb_rgb_camera_frame",
    )
    auto_white_balance_arg = DeclareLaunchArgument(
        "auto_white_balance",
        default_value="false",
    )
    white_balance_arg = DeclareLaunchArgument("white_balance", default_value="4000")
    autoexposure_arg = DeclareLaunchArgument("autoexposure", default_value="false")
    exposure_arg = DeclareLaunchArgument("exposure", default_value="100")
    gain_arg = DeclareLaunchArgument("gain", default_value="-1")
    brightness_arg = DeclareLaunchArgument("brightness", default_value="-1")
    contrast_arg = DeclareLaunchArgument("contrast", default_value="-1")
    saturation_arg = DeclareLaunchArgument("saturation", default_value="-1")
    sharpness_arg = DeclareLaunchArgument("sharpness", default_value="-1")

    target_color_arg = DeclareLaunchArgument("target_color", default_value="scene")
    hsv_ranges_json_arg = DeclareLaunchArgument("hsv_ranges_json", default_value="")
    min_area_px_arg = DeclareLaunchArgument("min_area_px", default_value="500")
    max_boxes_arg = DeclareLaunchArgument("max_boxes", default_value="20")
    basket_morph_close_kernel_size_arg = DeclareLaunchArgument(
        "basket_morph_close_kernel_size",
        default_value="51",
    )
    basket_min_area_px_arg = DeclareLaunchArgument(
        "basket_min_area_px",
        default_value="6000",
    )
    basket_max_area_px_arg = DeclareLaunchArgument(
        "basket_max_area_px",
        default_value="150000",
    )
    basket_min_bbox_width_px_arg = DeclareLaunchArgument(
        "basket_min_bbox_width_px",
        default_value="50",
    )
    basket_min_bbox_height_px_arg = DeclareLaunchArgument(
        "basket_min_bbox_height_px",
        default_value="70",
    )
    basket_min_aspect_ratio_arg = DeclareLaunchArgument(
        "basket_min_aspect_ratio",
        default_value="1.05",
    )
    basket_max_aspect_ratio_arg = DeclareLaunchArgument(
        "basket_max_aspect_ratio",
        default_value="5.0",
    )
    pose_smoothing_alpha_arg = DeclareLaunchArgument(
        "pose_smoothing_alpha",
        default_value="0.08",
        description="Raw center weight for temporal smoothing. 1.0 disables smoothing.",
    )
    yaw_smoothing_alpha_arg = DeclareLaunchArgument(
        "yaw_smoothing_alpha",
        default_value="0.05",
        description="Raw yaw weight for temporal smoothing. 1.0 disables smoothing.",
    )
    use_color_ratio_mask_arg = DeclareLaunchArgument(
        "use_color_ratio_mask",
        default_value="true",
    )
    use_color_quality_check_arg = DeclareLaunchArgument(
        "use_color_quality_check",
        default_value="true",
    )
    sort_by_arg = DeclareLaunchArgument("sort_by", default_value="x")
    plane_frame_arg = DeclareLaunchArgument("plane_frame", default_value="base")
    plane_z_m_arg = DeclareLaunchArgument("plane_z_m", default_value="0.0")
    plane_homography_arg = DeclareLaunchArgument(
        "plane_homography_json",
        default_value=DEFAULT_PLANE_HOMOGRAPHY_JSON,
        description="3x3 pixel-to-base homography JSON. Override if camera geometry changes.",
    )

    camera = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_rgb_camera",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_camera")),
        parameters=[
            {
                "video_device": LaunchConfiguration("video_device"),
                "image_width": ParameterValue(
                    LaunchConfiguration("image_width"),
                    value_type=int,
                ),
                "image_height": ParameterValue(
                    LaunchConfiguration("image_height"),
                    value_type=int,
                ),
                "framerate": ParameterValue(
                    LaunchConfiguration("framerate"),
                    value_type=float,
                ),
                "pixel_format": LaunchConfiguration("pixel_format"),
                "camera_frame_id": LaunchConfiguration("camera_frame_id"),
                "auto_white_balance": ParameterValue(
                    LaunchConfiguration("auto_white_balance"),
                    value_type=bool,
                ),
                "white_balance": ParameterValue(
                    LaunchConfiguration("white_balance"),
                    value_type=int,
                ),
                "autoexposure": ParameterValue(
                    LaunchConfiguration("autoexposure"),
                    value_type=bool,
                ),
                "exposure": ParameterValue(
                    LaunchConfiguration("exposure"),
                    value_type=int,
                ),
                "gain": ParameterValue(
                    LaunchConfiguration("gain"),
                    value_type=int,
                ),
                "brightness": ParameterValue(
                    LaunchConfiguration("brightness"),
                    value_type=int,
                ),
                "contrast": ParameterValue(
                    LaunchConfiguration("contrast"),
                    value_type=int,
                ),
                "saturation": ParameterValue(
                    LaunchConfiguration("saturation"),
                    value_type=int,
                ),
                "sharpness": ParameterValue(
                    LaunchConfiguration("sharpness"),
                    value_type=int,
                ),
            }
        ],
    )

    box_pose = Node(
        package="idle_vision",
        executable="box_pose_node",
        name="usb_rgb_box_pose_node",
        output="screen",
        parameters=[
            {
                "color_topic": "/image_raw",
                "depth_topic": "/idle_vision/no_depth",
                "camera_info_topic": "/camera_info",
                "target_color": ParameterValue(
                    LaunchConfiguration("target_color"),
                    value_type=str,
                ),
                "hsv_ranges_json": ParameterValue(
                    LaunchConfiguration("hsv_ranges_json"),
                    value_type=str,
                ),
                "min_area_px": ParameterValue(
                    LaunchConfiguration("min_area_px"),
                    value_type=int,
                ),
                "max_boxes": ParameterValue(
                    LaunchConfiguration("max_boxes"),
                    value_type=int,
                ),
                "basket_morph_close_kernel_size": ParameterValue(
                    LaunchConfiguration("basket_morph_close_kernel_size"),
                    value_type=int,
                ),
                "basket_min_area_px": ParameterValue(
                    LaunchConfiguration("basket_min_area_px"),
                    value_type=int,
                ),
                "basket_max_area_px": ParameterValue(
                    LaunchConfiguration("basket_max_area_px"),
                    value_type=int,
                ),
                "basket_min_bbox_width_px": ParameterValue(
                    LaunchConfiguration("basket_min_bbox_width_px"),
                    value_type=int,
                ),
                "basket_min_bbox_height_px": ParameterValue(
                    LaunchConfiguration("basket_min_bbox_height_px"),
                    value_type=int,
                ),
                "basket_min_aspect_ratio": ParameterValue(
                    LaunchConfiguration("basket_min_aspect_ratio"),
                    value_type=float,
                ),
                "basket_max_aspect_ratio": ParameterValue(
                    LaunchConfiguration("basket_max_aspect_ratio"),
                    value_type=float,
                ),
                "pose_smoothing_alpha": ParameterValue(
                    LaunchConfiguration("pose_smoothing_alpha"),
                    value_type=float,
                ),
                "yaw_smoothing_alpha": ParameterValue(
                    LaunchConfiguration("yaw_smoothing_alpha"),
                    value_type=float,
                ),
                "use_color_ratio_mask": ParameterValue(
                    LaunchConfiguration("use_color_ratio_mask"),
                    value_type=bool,
                ),
                "use_color_quality_check": ParameterValue(
                    LaunchConfiguration("use_color_quality_check"),
                    value_type=bool,
                ),
                "use_depth_candidate_mask": False,
                "require_depth": False,
                "base_frame": "",
                "sort_by": ParameterValue(
                    LaunchConfiguration("sort_by"),
                    value_type=str,
                ),
                "plane_frame": ParameterValue(
                    LaunchConfiguration("plane_frame"),
                    value_type=str,
                ),
                "plane_z_m": ParameterValue(
                    LaunchConfiguration("plane_z_m"),
                    value_type=float,
                ),
                "plane_homography_json": ParameterValue(
                    LaunchConfiguration("plane_homography_json"),
                    value_type=str,
                ),
            }
        ],
    )

    debug_view = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="rqt_usb_rgb_box_debug",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_rqt")),
        arguments=["/idle_vision/box_pose/debug_image"],
    )
    color_view = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="rqt_usb_rgb_color",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_rqt")),
        arguments=["/image_raw"],
    )

    return LaunchDescription(
        [
            start_camera_arg,
            start_rqt_arg,
            video_device_arg,
            image_width_arg,
            image_height_arg,
            framerate_arg,
            pixel_format_arg,
            camera_frame_id_arg,
            auto_white_balance_arg,
            white_balance_arg,
            autoexposure_arg,
            exposure_arg,
            gain_arg,
            brightness_arg,
            contrast_arg,
            saturation_arg,
            sharpness_arg,
            target_color_arg,
            hsv_ranges_json_arg,
            min_area_px_arg,
            max_boxes_arg,
            basket_morph_close_kernel_size_arg,
            basket_min_area_px_arg,
            basket_max_area_px_arg,
            basket_min_bbox_width_px_arg,
            basket_min_bbox_height_px_arg,
            basket_min_aspect_ratio_arg,
            basket_max_aspect_ratio_arg,
            pose_smoothing_alpha_arg,
            yaw_smoothing_alpha_arg,
            use_color_ratio_mask_arg,
            use_color_quality_check_arg,
            sort_by_arg,
            plane_frame_arg,
            plane_z_m_arg,
            plane_homography_arg,
            camera,
            box_pose,
            debug_view,
            color_view,
        ]
    )
