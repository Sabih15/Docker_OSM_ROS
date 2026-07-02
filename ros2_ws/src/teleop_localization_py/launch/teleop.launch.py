"""Launch the localization publisher (Python or C++) + the WebSocket bridge.

Usage:
    ros2 launch teleop_localization_py teleop.launch.py            # Python node
    ros2 launch teleop_localization_py teleop.launch.py impl:=cpp  # C++ node

The publish and broadcast rate is fixed at 30 Hz to match the requirement.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

RATE_HZ = 30.0  # fixed stream rate

def generate_launch_description() -> LaunchDescription:
    impl = LaunchConfiguration("impl")
    # True when impl == "cpp" (string comparison at launch time).
    use_cpp = PythonExpression(["'", impl, "' == 'cpp'"])

    common_params = [{
        "center_lat": 52.139200,   # OvGU Universitaetsplatz
        "center_lon": 11.645200,
        "rate_hz": RATE_HZ,
        "max_linear": 0.2,
        "max_angular": 0.2,
    }]

    return LaunchDescription([
        DeclareLaunchArgument(
            "impl", default_value="py",
            description="Localization node implementation: 'py' or 'cpp'."),

        # Python localization node (runs unless impl:=cpp)
        Node(
            package="teleop_localization_py",
            executable="localization_node",
            name="localization_node",
            output="screen",
            parameters=common_params,
            condition=UnlessCondition(use_cpp),
        ),

        # C++ localization node (runs only when impl:=cpp)
        Node(
            package="teleop_localization_cpp",
            executable="localization_node",
            name="localization_node",
            output="screen",
            parameters=common_params,
            condition=IfCondition(use_cpp),
        ),

        # WebSocket bridge (always Python)
        Node(
            package="teleop_localization_py",
            executable="ws_bridge",
            name="ws_bridge",
            output="screen",
            parameters=[{
                "gps_topic": "/robot/gps",
                "twist_topic": "/cmd_vel",
                "ws_host": "0.0.0.0",
                "ws_port": 9090,
                "ws_rate_hz": RATE_HZ,
            }],
        ),
    ])
