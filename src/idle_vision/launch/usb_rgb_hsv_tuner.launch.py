"""Launch a USB RGB camera and HSV ROI tuner."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DEFAULT_VIDEO_DEVICE = os.getenv("IDLE_VISION_VIDEO_DEVICE", "/dev/video2")


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

    color_topic_arg = DeclareLaunchArgument("color_topic", default_value="/image_raw")
    sample_u_arg = DeclareLaunchArgument("sample_u", default_value="-1")
    sample_v_arg = DeclareLaunchArgument("sample_v", default_value="-1")
    roi_half_size_px_arg = DeclareLaunchArgument("roi_half_size_px", default_value="12")
    publish_every_n_arg = DeclareLaunchArgument("publish_every_n", default_value="5")
    hsv_ranges_json_arg = DeclareLaunchArgument("hsv_ranges_json", default_value="")

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

    tuner = Node(
        package="idle_vision",
        executable="hsv_tuner_node",
        name="usb_rgb_hsv_tuner_node",
        output="screen",
        parameters=[
            {
                "color_topic": ParameterValue(
                    LaunchConfiguration("color_topic"),
                    value_type=str,
                ),
                "sample_u": ParameterValue(
                    LaunchConfiguration("sample_u"),
                    value_type=int,
                ),
                "sample_v": ParameterValue(
                    LaunchConfiguration("sample_v"),
                    value_type=int,
                ),
                "roi_half_size_px": ParameterValue(
                    LaunchConfiguration("roi_half_size_px"),
                    value_type=int,
                ),
                "publish_every_n": ParameterValue(
                    LaunchConfiguration("publish_every_n"),
                    value_type=int,
                ),
                "hsv_ranges_json": ParameterValue(
                    LaunchConfiguration("hsv_ranges_json"),
                    value_type=str,
                ),
                "publish_debug": True,
            }
        ],
    )

    debug_view = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="rqt_usb_rgb_hsv_debug",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_rqt")),
        arguments=["/idle_vision/hsv_tuner/debug_image"],
    )
    mask_view = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="rqt_usb_rgb_hsv_mask",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_rqt")),
        arguments=["/idle_vision/hsv_tuner/mask"],
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
            color_topic_arg,
            sample_u_arg,
            sample_v_arg,
            roi_half_size_px_arg,
            publish_every_n_arg,
            hsv_ranges_json_arg,
            camera,
            tuner,
            debug_view,
            mask_view,
        ]
    )
