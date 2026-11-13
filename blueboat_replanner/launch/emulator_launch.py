"""Detection emulator launch (eval/ only; run INSTEAD of Perception).
[Run on 2026-11-12 on an isolated ROS graph: test/ros_graph_check.py; never in Gazebo.]

    ros2 launch blueboat_replanner emulator_launch.py world_dir:=$HOME/worlds/<path>/<world> \
        seed:=1 calib_a:=<a> calib_b:=<b> [class_info:=true]

`resolution_m`, `sector_count`, `sector_origin_deg` and `look_timeout_s` are Belief's grid and
look parameters; their defaults are Belief's defaults. Change them only together with Belief's
configuration. `sensor_depth_m` (0.22 m) and `lever_arm_m` (0.285 m) are the sonar's mounting
as the processor and Belief use it.
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
    # name -> (default, type); explicit types so `calib_a:=2` stays a float (launch_ros would
    # YAML-parse it to an int and the node's float parameter would refuse it)
    args = {"world_dir": ("", str), "seed": ("1", int), "class_info": ("none", str),
            "calib_a": ("1.0", float), "calib_b": ("0.0", float), "range_m": ("20.0", float),
            "resolution_m": ("0.5", float), "sector_count": ("12", int),
            "sector_origin_deg": ("0.0", float), "look_timeout_s": ("20.0", float),
            "sensor_depth_m": ("0.22", float), "lever_arm_m": ("0.285", float)}
    decl = [DeclareLaunchArgument(k, default_value=v) for k, (v, _) in args.items()]
    node = Node(package="blueboat_replanner", executable="replanner_emulator_node",
                name="replanner_emulator", output="screen",
                parameters=[{k: ParameterValue(LaunchConfiguration(k), value_type=t)
                             for k, (_, t) in args.items()}])
    return LaunchDescription(decl + [node])
