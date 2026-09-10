#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


_ARGS = [
    ("robot_ns", "", "Namespace for the Livox and FAST-LIO nodes and topics"),
    ("use_sim_time", "false", "Use the simulation clock", ["true", "false"]),
    ("use_livox_driver", "true", "Start the Livox MID360s driver", ["true", "false"]),
    ("config_file", "mid360.yaml", "FAST-LIO configuration filename"),
    ("rviz", "false", "Start the FAST-LIO RViz configuration", ["true", "false"]),
]


def generate_launch_description():
    config_path = LaunchConfiguration("config_path")
    rviz_cfg = LaunchConfiguration("rviz_cfg")

    declared_args = [
        DeclareLaunchArgument(
            name,
            default_value=default_value,
            description=description,
            **({"choices": choices[0]} if choices else {}),
        )
        for name, default_value, description, *choices in _ARGS
    ]
    declared_args.extend(
        [
            DeclareLaunchArgument(
                "config_path",
                default_value=PathJoinSubstitution([FindPackageShare("fast_lio"), "config"]),
                description="Directory containing the FAST-LIO configuration file",
            ),
            DeclareLaunchArgument(
                "rviz_cfg",
                default_value=PathJoinSubstitution([FindPackageShare("fast_lio"), "rviz", "fastlio.rviz"]),
                description="FAST-LIO RViz configuration file",
            ),
        ]
    )

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("fast_lio"), "launch", "mapping.launch.py"])
        ),
        launch_arguments={
            "robot_ns": LaunchConfiguration("robot_ns"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_livox_driver": LaunchConfiguration("use_livox_driver"),
            "config_path": config_path,
            "config_file": LaunchConfiguration("config_file"),
            "rviz": LaunchConfiguration("rviz"),
            "rviz_cfg": rviz_cfg,
        }.items(),
    )

    return LaunchDescription([*declared_args, fast_lio_launch])
