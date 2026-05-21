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
    use_dashboard = LaunchConfiguration("use_dashboard")

    common_params = {
        "meters_per_world_unit": LaunchConfiguration("meters_per_world_unit"),
        "reference_lat": LaunchConfiguration("reference_lat"),
        "reference_lon": LaunchConfiguration("reference_lon"),
    }

    return LaunchDescription(
        [
            # --- Gazebo ---
            DeclareLaunchArgument("use_gazebo", default_value="true"),
            DeclareLaunchArgument("gz_args", default_value="-r -v 3"),

            # --- GPS & skala ---
            DeclareLaunchArgument("meters_per_world_unit", default_value="35.0"),
            DeclareLaunchArgument("reference_lat", default_value="-6.89148"),
            DeclareLaunchArgument("reference_lon", default_value="107.61066"),

            # --- Hiker ---
            DeclareLaunchArgument("gps_noise_std_m", default_value="2.0"),
            DeclareLaunchArgument(
                "trail_name",
                default_value="ridge_route",
                choices=["ridge_route", "valley_route", "crater_route"],
            ),
            DeclareLaunchArgument("hiker_speed_world_units_s", default_value="0.85"),

            # --- LoRa RF ---
            DeclareLaunchArgument("tx_power_dbm", default_value="17.0"),
            # Fitur 5: Spreading Factor (7–12, default 9)
            DeclareLaunchArgument("spreading_factor", default_value="9"),
            # Fitur 7: Fading model
            DeclareLaunchArgument("fading_model", default_value="rayleigh"),
            DeclareLaunchArgument("rician_k_db", default_value="10.0"),
            # Fitur 10: Cuaca
            DeclareLaunchArgument(
                "weather",
                default_value="clear",
                choices=["clear", "fog", "light_rain", "heavy_rain", "thunderstorm"],
            ),

            # --- Fitur 8: Baterai ---
            DeclareLaunchArgument("battery_capacity_mah", default_value="3000.0"),
            DeclareLaunchArgument("tx_current_ma", default_value="120.0"),
            DeclareLaunchArgument("idle_current_ma", default_value="3.0"),

            # --- Fitur 9: Rute dinamis ---
            DeclareLaunchArgument("routes_file", default_value=""),

            # --- Fitur 6: Dashboard ---
            DeclareLaunchArgument("use_dashboard", default_value="false"),
            DeclareLaunchArgument("dashboard_refresh_hz", default_value="0.5"),

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
                        "speed_world_units_s": LaunchConfiguration("hiker_speed_world_units_s"),
                        "trail_name": trail_name,
                        "gazebo_pose_control": use_gazebo,
                        # Fitur 8
                        "battery_capacity_mah": LaunchConfiguration("battery_capacity_mah"),
                        "tx_current_ma": LaunchConfiguration("tx_current_ma"),
                        "idle_current_ma": LaunchConfiguration("idle_current_ma"),
                        # Fitur 9
                        "routes_file": LaunchConfiguration("routes_file"),
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
                        # Fitur 5
                        "spreading_factor": LaunchConfiguration("spreading_factor"),
                        # Fitur 7
                        "fading_model": LaunchConfiguration("fading_model"),
                        "rician_k_db": LaunchConfiguration("rician_k_db"),
                        # Fitur 9
                        "routes_file": LaunchConfiguration("routes_file"),
                        # Fitur 10
                        "weather": LaunchConfiguration("weather"),
                    },
                ],
            ),
            Node(
                package="hiking_lora_sim",
                executable="base_station_display",
                output="screen",
            ),
            # Fitur 6: Dashboard (opsional)
            Node(
                package="hiking_lora_sim",
                executable="dashboard",
                output="screen",
                condition=IfCondition(use_dashboard),
                parameters=[
                    {"refresh_rate_hz": LaunchConfiguration("dashboard_refresh_hz")},
                ],
            ),
        ]
    )
