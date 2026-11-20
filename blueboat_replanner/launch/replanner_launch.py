"""Replanner node launch. [Run on 2026-11-12 on an isolated ROS graph: test/ros_graph_check.py.]

    ros2 launch blueboat_replanner replanner_launch.py mission_file:=/abs/mission.yaml \
        [config_file:=/abs/replanner.yaml] [c2_tuned_file:=/abs/c2_tuned.yaml] [data_dir:=...] \
        [belief_topic:=/belief_map/belief_grid] [use_sim_time:=true time_source:=ros]

`use_sim_time` (default false) and `time_source` (`wall`, default, or `ros`): in Gazebo pass both,
`use_sim_time:=true time_source:=ros`, so every age, timeout and record time is simulated seconds
(the ray-cast run is slowed below real time and paused while a slow ping renders); on the boat keep
the defaults. `time_source:=ros` without `use_sim_time` is the system time, the same as `wall`
(MATH §11.3).

`c2_tuned_file` defaults to the installed `config/c2_tuned.yaml` (the C2 freeze every mission
flies; io/mission.py).

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
    args = {"mission_file": ("", str), "config_file": ("", str), "c2_tuned_file": ("", str),
            "data_dir": ("", str),
            "belief_topic": ("/belief_map/belief_grid", str), "tick_hz": ("5.0", float),
            "use_sim_time": ("false", bool), "time_source": ("wall", str)}
    decl = [DeclareLaunchArgument(k, default_value=v) for k, (v, _) in args.items()]
    node = Node(package="blueboat_replanner", executable="replanner_node", name="replanner",
                output="screen",
                parameters=[{k: ParameterValue(LaunchConfiguration(k), value_type=t)
                             for k, (_, t) in args.items()}])
    return LaunchDescription(decl + [node])
