"""Detection emulator launch (eval/ only; run INSTEAD of Perception). [NOT RUN.]

    ros2 launch blueboat_replanner emulator_launch.py world_dir:=$HOME/worlds/<path>/<world> \
        seed:=1 calib_a:=<a> calib_b:=<b>
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
    args = {"world_dir": ("", str), "seed": ("1", int), "output": ("detections", str),
            "calib_a": ("1.0", float), "calib_b": ("0.0", float), "range_m": ("20.0", float)}
    decl = [DeclareLaunchArgument(k, default_value=v) for k, (v, _) in args.items()]
    node = Node(package="blueboat_replanner", executable="replanner_emulator_node",
                name="replanner_emulator", output="screen",
                parameters=[{k: ParameterValue(LaunchConfiguration(k), value_type=t)
                             for k, (_, t) in args.items()}])
    return LaunchDescription(decl + [node])
