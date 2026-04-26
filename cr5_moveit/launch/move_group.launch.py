from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")

    moveit_config = MoveItConfigsBuilder("cr5_robot", package_name="cr5_moveit").to_moveit_configs()
    inner = generate_move_group_launch(moveit_config)

    def _expand(context, *args, **kwargs):
        ns = str(namespace.perform(context) or "").strip().strip("/")
        actions = []
        if ns:
            actions.append(PushRosNamespace(ns))
        actions.extend(list(getattr(inner, "entities", [])))
        return actions

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            OpaqueFunction(function=_expand),
        ]
    )
