"""Replanner node launch. [NOT RUN — needs in-machine session.]

    ros2 launch blueboat_replanner replanner_launch.py mission_file:=/abs/mission.yaml \
        [config_file:=/abs/replanner.yaml] [data_dir:=...] [belief_topic:=/belief_map/belief_grid]

ROS launch symbols are imported optionally so the offline gate can compile this file.
"""

try:
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node
    from launch_ros.parameter_descriptions import ParameterValue
except ImportError:          # offline gate
    LaunchDescription = None


def generate_launch_description():
    # name -> (default, type); explicit types so an empty path stays a string and
    # `tick_hz:=5` stays a float (launch_ros YAML-parses argument strings otherwise)
    args = {"mission_file": ("", str), "config_file": ("", str), "data_dir": ("", str),
            "belief_topic": ("/belief_map/belief_grid", str), "tick_hz": ("5.0", float)}
    decl = [DeclareLaunchArgument(k, default_value=v) for k, (v, _) in args.items()]
    node = Node(package="blueboat_replanner", executable="replanner_node", name="replanner",
                output="screen",
                parameters=[{k: ParameterValue(LaunchConfiguration(k), value_type=t)
                             for k, (_, t) in args.items()}])
    return LaunchDescription(decl + [node])
