#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    Command,
    EnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

# ---------------------------------------------------------------------------
# Argument declarations  (name, default, description, (optional) choices)
# ---------------------------------------------------------------------------
_ARGS = [
    ("robot_model", "hydrus", "Name of the robot model ROS package"),
    ("robot_ns", "hydrus", "Namespace for all robot nodes"),
    ("real_machine", "true", "Use real machine specific bring-up inside model_launch", ["true", "false"]),
    ("headless", "true", "Run without GUI", ["true", "false"]),
    ("sim", "false", "Launch Gazebo simulation", ["true", "false"]),
    ("rviz_config_path", "rviz_config.rviz", "RViz config filename (resolved inside robot_model pkg/config/)"),
    ("rviz_init_path", "RvizInit.yaml", "Parameter file with initial joint values for joint_state_publisher_gui"),
]


def gui_check(context, *args, **kwargs):
    headless = LaunchConfiguration("headless").perform(context)
    display = EnvironmentVariable("DISPLAY", default_value="").perform(context)
    wayland_display = EnvironmentVariable("WAYLAND_DISPLAY", default_value="").perform(context)
    xauthority = EnvironmentVariable("XAUTHORITY", default_value="").perform(context)
    gui_available = PythonExpression(
        [
            "'true' if (len('",
            wayland_display,
            "') > 0) or ((len('",
            display,
            "') > 0) and (len('",
            xauthority,
            "') > 0)) else 'false'",
        ]
    ).perform(context)

    if headless.lower() == "false" and gui_available.lower() == "false":
        raise RuntimeError("[ERROR] [launch]: Headless mode is deactivated but no GUI environment detected. \
                           Set headless to true or ensure DISPLAY and XAUTHORITY (X11) or \
                           WAYLAND_DISPLAY (Wayland) are set in the environment.")
    if headless.lower() == "false" and gui_available.lower() == "true":
        if len(wayland_display) > 0:
            print("[INFO] [launch]: GUI environment detected: Wayland (WAYLAND_DISPLAY is set)")
        elif len(display) > 0 and len(xauthority) > 0:
            print("[INFO] [launch]: GUI environment detected: X11 (DISPLAY and XAUTHORITY are set)")


def generate_launch_description():
    # ------------------------------------------------------------------
    # 1.  Declare CLI-overridable arguments
    # ------------------------------------------------------------------
    declared_args = [
        DeclareLaunchArgument(
            name, default_value=default_value, description=description, **({"choices": choices[0]} if choices else {})
        )
        for name, default_value, description, *choices in _ARGS
    ]

    # Resolve / Read at launch time (NOT AT IMPORT TIME)
    robot_model_pkg = LaunchConfiguration("robot_model")
    robot_ns = LaunchConfiguration("robot_ns")
    real_machine = LaunchConfiguration("real_machine")
    headless = LaunchConfiguration("headless")
    sim = LaunchConfiguration("sim")
    rviz_config_path = LaunchConfiguration("rviz_config_path")
    rviz_init_path = LaunchConfiguration("rviz_init_path")
    robot_description = LaunchConfiguration("robot_description")

    # TF frame ids are not affected by a ROS node namespace.  Normalize the
    # robot namespace separately and use it as the frame prefix for every
    # robot-owned frame.
    tf_prefix = PythonExpression(["'", robot_ns, "'.strip('/')"])
    robot_state_publisher_frame_prefix = PythonExpression(
        ["('", robot_ns, "'.strip('/') + '/') if '", robot_ns, "'.strip('/') else ''"]
    )

    # Guard against RViz/Qt crashes in headless environments (containers, SSH without X11/Wayland).
    # We consider a GUI available if:
    # - Wayland: WAYLAND_DISPLAY is set, OR
    # - X11: DISPLAY and XAUTHORITY are both set.
    display = EnvironmentVariable("DISPLAY", default_value="")
    wayland_display = EnvironmentVariable("WAYLAND_DISPLAY", default_value="")
    xauthority = EnvironmentVariable("XAUTHORITY", default_value="")
    gui_available = PythonExpression(
        [
            "'true' if (len('",
            wayland_display,
            "') > 0) or ((len('",
            display,
            "') > 0) and (len('",
            xauthority,
            "') > 0)) else 'false'",
        ]
    )
    launch_gui = PythonExpression(["('", headless, "' == 'false') and ('", gui_available, "' == 'true')"])

    robot_description_param = {"robot_description": ParameterValue(Command(robot_description), value_type=str)}

    # TODO WHAT IS THIS!? how can we have both sim and real_machine false at the same time? What does this represent? Necessary?
    need_joint_states = PythonExpression(
        ["'false' if '", sim, "' == 'true' or '", real_machine, "' == 'true' else 'true'"]
    )

    # ------------------------------------------------------------------
    # 2.  Nodes
    # ------------------------------------------------------------------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_ns,
        parameters=[
            {"frame_prefix": robot_state_publisher_frame_prefix, "use_sim_time": sim},
            robot_description_param,
        ],
    )

    rotor_tf_publisher_node = Node(
        package="aerial_robot_model",
        executable="rotor_tf_publisher",
        name="rotor_tf_publisher",
        namespace=robot_ns,
        condition=UnlessCondition(need_joint_states),
        parameters=[{"tf_prefix": tf_prefix, "use_sim_time": sim}, robot_description_param],
    )

    joint_state_pub_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        namespace=robot_ns,
        condition=IfCondition(need_joint_states),
        parameters=[{"use_sim_time": sim}, rviz_init_path, robot_description_param],
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=robot_ns,
        arguments=["-d", rviz_config_path],
        condition=IfCondition(launch_gui),
        parameters=[{"use_sim_time": sim}],
        output="screen",
    )

    # ------------------------------------------------------------------
    # 3.  Assemble LaunchDescription
    # ------------------------------------------------------------------
    ld = LaunchDescription()

    for arg in declared_args:
        ld.add_action(arg)

    ld.add_action(OpaqueFunction(function=gui_check))
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rotor_tf_publisher_node)
    ld.add_action(joint_state_pub_node)
    ld.add_action(rviz2_node)

    return ld
