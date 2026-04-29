from moveit_configs_utils import MoveItConfigsBuilder

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from moveit_configs_utils.launch_utils import add_debuggable_node, DeclareBooleanLaunchArg
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os
import yaml


def load_yaml(package_name, file_path):
    package_path = Path(get_package_share_directory(package_name))
    absolute_file_path = package_path / file_path
    with absolute_file_path.open("r", encoding="utf-8") as file:
        return yaml.safe_load(file) or {}


def normalize_pipeline_params(params):
    normalized = dict(params)
    for key in ("request_adapters", "capabilities"):
        value = normalized.get(key)
        if isinstance(value, str):
            normalized[key] = ParameterValue(value, value_type=str)
    return normalized


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    pilz_config_name = LaunchConfiguration("pilz_config_name")

    def _expand(context, *args, **kwargs):
        ns = str(namespace.perform(context) or "").strip().strip("/")
        config_name = str(pilz_config_name.perform(context) or "").strip()
        config_file = f"config/{config_name}"

        moveit_config = MoveItConfigsBuilder("cr5_robot", package_name="cr5_moveit").to_moveit_configs()
        raw_pipeline_params = load_yaml("cr5_moveit", config_file)
        pipeline_params = {
            "pilz_industrial_motion_planner": normalize_pipeline_params(raw_pipeline_params)
        }

        inner = build_move_group_launch(moveit_config, pipeline_params)
        actions = []
        if ns:
            actions.append(PushRosNamespace(ns))
        actions.extend(list(getattr(inner, "entities", [])))
        return actions

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument(
                "pilz_config_name",
                default_value="pilz_industrial_motion_planner_planning.yaml",
            ),
            OpaqueFunction(function=_expand),
        ]
    )


def build_move_group_launch(moveit_config, pipeline_params=None):
    pipeline_params = pipeline_params or {}

    ld = LaunchDescription()
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )
    ld.add_action(
        DeclareLaunchArgument(
            "capabilities",
            default_value=moveit_config.move_group_capabilities["capabilities"],
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "disable_capabilities",
            default_value=moveit_config.move_group_capabilities["disable_capabilities"],
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }

    move_group_params = [
        moveit_config.to_dict(),
        pipeline_params,
        move_group_configuration,
    ]

    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        commands_file=str(moveit_config.package_path / "launch" / "gdb_settings.gdb"),
        output="screen",
        parameters=move_group_params,
        extra_debug_args=["--debug"],
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
    )
    return ld
