from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("hiking_lora_sim")
    trail_name = LaunchConfiguration("trail_name")
    world_file = PythonExpression(["'hiking_mountain_' + '", trail_name, "' + '.sdf'"])
    world_path = PathJoinSubstitution([pkg_share, "worlds", world_file])
    ros_gz_sim_launch = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )

    use_gazebo = LaunchConfiguration("use_gazebo")
    gz_args = LaunchConfiguration("gz_args")

    common_params = {
        "meters_per_world_unit": LaunchConfiguration("meters_per_world_unit"),
        "reference_lat": LaunchConfiguration("reference_lat"),
        "reference_lon": LaunchConfiguration("reference_lon"),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_gazebo", default_value="true"),
            DeclareLaunchArgument("gz_args", default_value="-r -v 3"),
            DeclareLaunchArgument("meters_per_world_unit", default_value="35.0"),
            DeclareLaunchArgument("reference_lat", default_value="-6.89148"),
            DeclareLaunchArgument("reference_lon", default_value="107.61066"),
            DeclareLaunchArgument("gps_noise_std_m", default_value="2.0"),
            DeclareLaunchArgument("trail_name", default_value="ridge_route"),
            DeclareLaunchArgument("tx_power_dbm", default_value="17.0"),
            DeclareLaunchArgument("receiver_sensitivity_dbm", default_value="-126.0"),
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                [
                    pkg_share,
                    ":",
                    PathJoinSubstitution([pkg_share, "models"]),
                    ":",
                    EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ros_gz_sim_launch),
                condition=IfCondition(use_gazebo),
                launch_arguments={"gz_args": [gz_args, " ", world_path]}.items(),
            ),
            Node(
                package="hiking_lora_sim",
                executable="hiker_agent",
                output="screen",
                parameters=[
                    common_params,
                    {
                        "gps_noise_std_m": LaunchConfiguration("gps_noise_std_m"),
                        "trail_name": trail_name,
                    },
                ],
            ),
            Node(
                package="hiking_lora_sim",
                executable="lora_network",
                output="screen",
                parameters=[
                    common_params,
                    {
                        "tx_power_dbm": LaunchConfiguration("tx_power_dbm"),
                        "receiver_sensitivity_dbm": LaunchConfiguration(
                            "receiver_sensitivity_dbm"
                        ),
                    },
                ],
            ),
            Node(
                package="hiking_lora_sim",
                executable="base_station_display",
                output="screen",
            ),
        ]
    )
