#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# ---------------------------------------------------------------------------
# Argument declarations  (name, default, description, (optional) choices)
# ---------------------------------------------------------------------------
_ARGS = [
    ("robot_ns", "/", "Namespace for all robot nodes"),
    ("heartbeat_rate_hz", "2.0", "GCS-to-FC health heartbeat rate"),
]


def generate_launch_description():
    # ------------------------------------------------------------------
    # 1.  Declare CLI-overridable arguments
    # ------------------------------------------------------------------
    declared_args = [
        DeclareLaunchArgument(
            name,
            default_value=default_value,
            description=description,
            **({"choices": choices[0]} if choices else {})
        )
        for name, default_value, description, *choices in _ARGS
    ]

    # Resolve / Read at launch time (NOT AT IMPORT TIME)
    robot_ns = LaunchConfiguration("robot_ns")
    heartbeat_rate_hz = LaunchConfiguration("heartbeat_rate_hz")

    # ------------------------------------------------------------------
    # 2.  Nodes
    # ------------------------------------------------------------------
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        namespace=robot_ns,
        output="screen",
        parameters=[
            {
                "dev": "/dev/input/js0",
                "coalesce_interval": 0.025,
            }
        ],
    )

    heartbeat_node = Node(
        package="aerial_robot_core",
        executable="network_heartbeat.py",
        name="network_heartbeat",
        namespace=robot_ns,
        output="screen",
        parameters=[{"rate_hz": ParameterValue(heartbeat_rate_hz, value_type=float)}],
    )

    # ------------------------------------------------------------------
    # 3.  Assemble LaunchDescription
    # ------------------------------------------------------------------
    ld = LaunchDescription()

    for arg in declared_args:
        ld.add_action(arg)

    ld.add_action(joy_node)
    ld.add_action(heartbeat_node)
    return ld
