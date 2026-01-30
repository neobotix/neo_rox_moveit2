# Author: Pradheep Padmanabhan
# contributor: Ashin Anandakrishnan
import os

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

def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    rox_type = LaunchConfiguration("rox_type")
    arm_type = LaunchConfiguration("arm_type")
    gripper_type = LaunchConfiguration("gripper_type")
    use_mock = LaunchConfiguration("use_mock_hardware")

    # General arguments
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_gz = LaunchConfiguration("use_gz")
    simulation_enabled = context.perform_substitution(LaunchConfiguration("use_gz")).lower()

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
        "fr3_controllers.yaml",
    )
    controllers_yaml_with_substitutions = ParameterFile(controllers_yaml, allow_substs=True)
    # Evaluate the parameter file to apply dynamic substitutions
    controllers_yaml_with_substitutions.evaluate(context)
    
    # Load the controllers YAML
    controllers_yaml_dict = load_yaml(
        str(moveit_config_package.perform(context)),
        str(controllers_yaml_with_substitutions.param_file)
    )

    # Franka uses joint_trajectory_controller by default
    # Adjust controller configuration based on hardware mode
    use_mock_hardware = context.perform_substitution(LaunchConfiguration("use_mock_hardware")).lower()
    if use_mock_hardware == "true" or simulation_enabled == "true":
        # For mock/simulation, ensure the standard controller is used
        if "moveit_simple_controller_manager" in controllers_yaml_dict:
            if "joint_trajectory_controller" in controllers_yaml_dict["moveit_simple_controller_manager"]:
                controllers_yaml_dict["moveit_simple_controller_manager"]["joint_trajectory_controller"]["default"] = True

    # Joint Limits Configuration
    joint_limits_yaml = os.path.join(
        get_package_share_directory(str(moveit_config_package.perform(context))),
        "config",
        "franka_joint_limits.yaml",
    )
    joint_limits_yaml_with_substitutions = ParameterFile(joint_limits_yaml, allow_substs=True)
    # Evaluate the parameter file to apply dynamic substitutions
    joint_limits_yaml_with_substitutions.evaluate(context)
    joint_limits_yaml = os.path.join(
        get_package_share_directory('neo_rox_moveit2'),
        "config",
        str(joint_limits_yaml_with_substitutions.param_file)
    )

    # Kinematics Configuration
    kinematics_yaml = os.path.join(
    get_package_share_directory(str(moveit_config_package.perform(context))),
    "config",
    "kinematics_franka.yaml",
    )
    kinematics_yaml_with_substitutions = ParameterFile(kinematics_yaml, allow_substs=True)
    # Evaluate the parameter file to apply dynamic substitutions
    kinematics_yaml_with_substitutions.evaluate(context)
    kinematics_yaml = os.path.join(
        get_package_share_directory('neo_rox_moveit2'),
        "config",
        str(kinematics_yaml_with_substitutions.param_file)
    )

    moveit_config = (
        MoveItConfigsBuilder(robot_name="rox_franka", package_name="neo_rox_moveit2")
        .robot_description_semantic(file_path=srdf, mappings={
            "prefix": prefix,
            "rox_type": rox_type,
            "gripper_type": gripper_type,
            })
        .robot_description(file_path=urdf, mappings={
            "use_gz": use_gz,
            "rox_type": rox_type,
            "arm_type": arm_type,
            "gripper_type": gripper_type,
            "force_abs_paths": use_gz,
            "use_mock_hardware": use_mock,
            "mock_sensor_commands": use_mock,
            })
        .joint_limits(file_path=joint_limits_yaml)
        .to_moveit_configs()
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
            kinematics_yaml,
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

    nodes_to_start = [move_group_node, rviz_node]

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
            default_value='fr3',
            description='Arm Types:\n'
                        '\t Franka Emika: fr3\n'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'gripper_type', 
            default_value='franka_hand',
            choices=['', 'franka_hand', '2f_140', '2f_85', 'epick'],
            description='Gripper Types - Supported Robots\n\t'
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
              '\t This is needed for the trajectory planning in simulation.\n'
              '\t Defaults to True if `use_gz` is True'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='',
            description='Prefix of the joint names in controllers configuration.\n'
            '\t (same as "arm_type" argument)',
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