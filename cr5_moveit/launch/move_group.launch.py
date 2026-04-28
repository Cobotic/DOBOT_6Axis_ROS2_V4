from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import yaml


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")

    def _expand(context, *args, **kwargs):
        ns = str(namespace.perform(context) or "").strip().strip("/")

        moveit_config = MoveItConfigsBuilder("cr5_robot", package_name="cr5_moveit").to_moveit_configs()
        if ns:
            old_group = "cr5_group"
            new_group = f"/{ns}/{old_group}"

            pkg_share = Path(get_package_share_directory("cr5_moveit"))

            srdf_path = pkg_share / "config" / "cr5_robot.srdf"
            srdf = srdf_path.read_text(encoding="utf-8")
            srdf = srdf.replace(f'name="{old_group}"', f'name="{new_group}"')
            srdf = srdf.replace(f'group="{old_group}"', f'group="{new_group}"')
            moveit_config.robot_description_semantic = {
                "robot_description_semantic": ParameterValue(srdf, value_type=str)
            }

            kin_path = pkg_share / "config" / "kinematics.yaml"
            kin_raw = yaml.safe_load(kin_path.read_text(encoding="utf-8")) or {}
            if isinstance(kin_raw, dict) and old_group in kin_raw and new_group not in kin_raw:
                kin_raw[new_group] = kin_raw.pop(old_group)
            moveit_config.robot_description_kinematics = {"robot_description_kinematics": kin_raw}

        inner = generate_move_group_launch(moveit_config)
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
