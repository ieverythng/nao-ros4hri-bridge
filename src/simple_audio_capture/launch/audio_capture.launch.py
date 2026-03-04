#!/usr/bin/env python3
"""Launch simple_audio_capture with optional topic override."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [FindPackageShare("simple_audio_capture"), "config", "speech_recognition.yaml"]
    )

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="Path to parameter YAML file.",
    )
    audio_topic_arg = DeclareLaunchArgument(
        "audio_topic",
        default_value="/laptop/microphone0",
        description="Audio topic name for AudioData publishing.",
    )

    node = Node(
        package="simple_audio_capture",
        executable="audio_capture_node",
        name="simple_audio_capture",
        output="screen",
        parameters=[
            LaunchConfiguration("config"),
            {"audio_topic": LaunchConfiguration("audio_topic")},
        ],
    )

    return LaunchDescription([config_arg, audio_topic_arg, node])
