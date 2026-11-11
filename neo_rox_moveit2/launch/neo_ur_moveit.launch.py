# Copyright (c) 2024 Neobotix GmbH.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Pradheep Padmanabhan
# Contributor: Adarsh Karan K P

import os
from copy import deepcopy

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterFile
from moveit_configs_utils import MoveItConfigsBuilder
from pathlib import Path
from neo_rox_moveit2.launch_common import load_yaml
from launch.event_handlers import OnProcessExit


UR_JOINT_SUFFIXES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


def configure_moveit_controllers(config, arm_prefixes, use_unscaled_controller):
    """Expand the single-arm controller templates for one or two UR arms."""
    manager = config["moveit_simple_controller_manager"]
    scaled_name = "scaled_joint_trajectory_controller"
    unscaled_name = "joint_trajectory_controller"

    if len(arm_prefixes) == 1:
        controller_pairs = [(scaled_name, unscaled_name)]
    else:
        scaled_template = manager.pop(scaled_name)
        unscaled_template = manager.pop(unscaled_name)
        manager["controller_names"] = [
            name
            for name in manager["controller_names"]
            if name not in (scaled_name, unscaled_name)
        ]
        controller_pairs = []

        for arm_name, joint_prefix in arm_prefixes:
            arm_scaled_name = f"{arm_name}_{scaled_name}"
            arm_unscaled_name = f"{arm_name}_{unscaled_name}"

            for controller_name, template in (
                (arm_scaled_name, scaled_template),
                (arm_unscaled_name, unscaled_template),
            ):
                controller = deepcopy(template)
                controller["joints"] = [
                    f"{joint_prefix}{joint}" for joint in UR_JOINT_SUFFIXES
                ]
                manager[controller_name] = controller
                manager["controller_names"].append(controller_name)

            controller_pairs.append((arm_scaled_name, arm_unscaled_name))

    for arm_scaled_name, arm_unscaled_name in controller_pairs:
        manager[arm_scaled_name]["default"] = not use_unscaled_controller
        manager[arm_unscaled_name]["default"] = use_unscaled_controller


def add_arm2_planning_parameters(moveit_config, arm1_prefix, arm2_prefix):
    """Reuse arm 1 kinematics and joint-limit settings for arm 2."""
    kinematics = moveit_config.robot_description_kinematics[
        "robot_description_kinematics"
    ]
    kinematics["ur_manipulator_arm2"] = deepcopy(kinematics["ur_manipulator"])

    joint_limits = moveit_config.joint_limits["robot_description_planning"][
        "joint_limits"
    ]
    for joint in UR_JOINT_SUFFIXES:
        joint_limits[f"{arm2_prefix}{joint}"] = deepcopy(
            joint_limits[f"{arm1_prefix}{joint}"]
        )


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    rox_type = LaunchConfiguration("rox_type")
    arm_type = LaunchConfiguration("arm_type")
    arm2_type = LaunchConfiguration("arm2_type")
    gripper_type = LaunchConfiguration("gripper_type")
    use_mock = LaunchConfiguration("use_mock_hardware")

    # General arguments
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    arm1_prefix = LaunchConfiguration("arm1_prefix")
    arm2_prefix = LaunchConfiguration("arm2_prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    ur_dc = LaunchConfiguration("use_ur_dc")
    use_gz = LaunchConfiguration("use_gz")
    simulation_enabled = context.perform_substitution(
        LaunchConfiguration("use_gz")
    ).lower()

    if simulation_enabled == "true":
        use_sim_time = True

    urdf = os.path.join(get_package_share_directory('rox_description'),
        'urdf',
        'rox.urdf.xacro')

    # MoveIt Configuration
    srdf = os.path.join(get_package_share_directory('neo_rox_moveit2'),
        'srdf',
        'rox.srdf.xacro')

    # Controllers Configuration
    controllers_yaml = os.path.join(
        get_package_share_directory(str(moveit_config_package.perform(context))),
        "config",
        "moveit_controllers.yaml",
    )
    controllers_yaml_with_substitutions = ParameterFile(
        controllers_yaml, allow_substs=True
    )
    # Evaluate the parameter file to apply dynamic substitutions
    controllers_yaml_with_substitutions.evaluate(context)
    
    # Load the controllers YAML
    controllers_yaml_dict = load_yaml(
        str(moveit_config_package.perform(context)),
        str(controllers_yaml_with_substitutions.param_file)
    )

    dual_arm_enabled = bool(arm2_type.perform(context))
    arm_prefixes = [("arm1", arm1_prefix.perform(context))]
    if dual_arm_enabled:
        arm_prefixes.append(("arm2", arm2_prefix.perform(context)))

    # The scaled_joint_trajectory_controller does not work on mock hardware.
    use_mock_hardware = context.perform_substitution(
        LaunchConfiguration("use_mock_hardware")
    ).lower()
    use_unscaled_controller = (
        use_mock_hardware == "true" or simulation_enabled == "true"
    )
    configure_moveit_controllers(
        controllers_yaml_dict, arm_prefixes, use_unscaled_controller
    )

    # Joint Limits Configuration
    joint_limits_yaml = os.path.join(
        get_package_share_directory(str(moveit_config_package.perform(context))),
        "config",
        "joint_limits.yaml",
    )
    joint_limits_yaml_with_substitutions = ParameterFile(
        joint_limits_yaml, allow_substs=True
    )
    # Evaluate the parameter file to apply dynamic substitutions
    joint_limits_yaml_with_substitutions.evaluate(context)
    joint_limits_yaml = os.path.join(
        get_package_share_directory('neo_rox_moveit2'),
        "config",
        str(joint_limits_yaml_with_substitutions.param_file)
    )

    moveit_config = (
        MoveItConfigsBuilder(robot_name="rox_ur", package_name="neo_rox_moveit2")
        .robot_description_semantic(file_path=srdf, mappings={
            "prefix": arm1_prefix,
            "arm1_prefix": arm1_prefix,
            "arm2_prefix": arm2_prefix,
            "arm2_type": arm2_type,
            "rox_type": rox_type,
            "gripper_type": gripper_type,
            })
        .robot_description(file_path=urdf, mappings={
            "use_gz": use_gz,
            "rox_type": rox_type,
            "arm_type": arm_type,
            "arm2_type": arm2_type,
            "arm1_prefix": arm1_prefix,
            "arm2_prefix": arm2_prefix,
            "use_ur_dc": ur_dc,
            "gripper_type": gripper_type,
            "force_abs_paths": use_gz,
            "use_mock_hardware": use_mock,
            "mock_sensor_commands": use_mock,
            })
        .joint_limits(file_path=joint_limits_yaml)
        .to_moveit_configs()
    )
    if dual_arm_enabled:
        add_arm2_planning_parameters(
            moveit_config,
            arm1_prefix.perform(context),
            arm2_prefix.perform(context),
        )
    # Override the trajectory_execution with the modified dictionary
    moveit_config.trajectory_execution = controllers_yaml_dict

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
    )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "rviz", "view_robot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    wait_robot_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="screen",
    )

    handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_robot_description,
            on_exit=[move_group_node, rviz_node],
        )
    )

    nodes_to_start = [wait_robot_description, handler]

    return nodes_to_start

def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'rox_type',
            default_value='argo',
            choices = ['', 'argo', 'argo-trio', 'diff', 'trike'],
            description='Robot type\n\t'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'arm_type', 
            default_value='',
            description='Arm Types:\n'
                        '\t Elite Arms: ec66, cs66\n'
                        '\t Universal Robotics (UR): ur5, ur10, ur5e, ur10e' 
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'arm2_type',
            default_value='',
            description='Optional second UR arm type. Leave empty for single-arm operation.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'gripper_type', default_value='',
            choices=['', '2f_140', '2f_85', 'epick'],
            description='Gripper Types - Supported Robots [mpo-700, mpo-500]\n\t'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="neo_rox_moveit2",
            description='MoveIt config package with robot SRDF/XACRO files. Usually the argument\n'
            '\t is not set, it enables use of a custom moveit config.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            description='Make MoveIt to use simulation time.\n'
              '\t This is needed for the trajectory planing in simulation.\n'
              '\t Defaults to True if `use_gz` is True'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='arm1_',
            description='Deprecated alias used as the default for arm1_prefix.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "arm1_prefix",
            default_value=LaunchConfiguration("prefix"),
            description='Joint and link prefix for arm 1.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "arm2_prefix",
            default_value='arm2_',
            description='Joint and link prefix for arm 2.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_ur_dc",
            default_value="False",
            description="Uses a shorter cabin box. Check with Neobotix, before selecting.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gz",
            default_value="False",
            description="Whether to enable Gazebo simulation.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="False",
            description="Indicate whether robot is running with mock hardware mirroring command to its states.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="True", description="Launch RViz?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
