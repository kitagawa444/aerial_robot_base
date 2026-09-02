#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("mini_quadrotor"), "launch", "bringup_launch.py"])
        ),
        launch_arguments={
            "real_machine": "false",
            "sim": "true",
            "headless": "true",
            "simulate_crsf": "true",
        }.items(),
    )

    test = Node(
        package="spinal",
        executable="crsf_gazebo_test.py",
        name="crsf_gazebo_test",
        namespace="mini_quadrotor",
        output="screen",
    )

    shutdown_when_finished = RegisterEventHandler(OnProcessExit(target_action=test, on_exit=[Shutdown()]))
    return LaunchDescription([bringup, test, shutdown_when_finished])
