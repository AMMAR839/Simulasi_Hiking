import os
import random
from math import ceil, hypot

from ament_index_python.packages import PackageNotFoundError, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from hiking_lora_sim.scenario import (
    ACTIVE_TRAIL_NAME,
    BASE_STATION,
    LORA_NODES,
    TRAILS,
    load_scenario_yaml,
    nearest_trail_progress,
    point_on_trail,
    terrain_height_world,
    trail_length,
)


_ROUTE_ALIASES = {
    "a": "ridge_route",
    "b": "valley_route",
    "c": "crater_route",
    "ridge": "ridge_route",
    "valley": "valley_route",
    "crater": "crater_route",
}

_BUILTIN_ROUTES = {"ridge_route", "valley_route", "crater_route"}


def _cfg(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def _active_trails(routes_file: str):
    scenario = load_scenario_yaml(routes_file) if routes_file else None
    if scenario and scenario.get("trails"):
        return scenario["trails"]
    return TRAILS


def _scenario_stations(routes_file: str):
    scenario = load_scenario_yaml(routes_file) if routes_file else None
    if scenario:
        lora_nodes = scenario.get("lora_nodes", LORA_NODES)
        base_station = scenario.get("base_station", BASE_STATION)
        return lora_nodes, base_station
    return LORA_NODES, BASE_STATION


def _resolve_route(raw_name: str, active_trails) -> str:
    name = raw_name.strip() or ACTIVE_TRAIL_NAME
    route_name = _ROUTE_ALIASES.get(name.lower(), name)
    if route_name not in active_trails:
        choices = ", ".join(sorted(active_trails))
        raise RuntimeError(
            f"Unknown jalur '{raw_name}'. Gunakan a/b/c atau salah satu route: {choices}"
        )
    return route_name


def _positive_int(raw_value: str, label: str) -> int:
    try:
        value = int(raw_value)
    except ValueError as exc:
        raise RuntimeError(f"{label} harus berupa angka integer.") from exc
    if value < 1:
        raise RuntimeError(f"{label} minimal 1.")
    return value


def _parse_hiker_routes(context, active_trails):
    pendaki_routes = _cfg(context, "pendaki_routes").strip()
    if pendaki_routes:
        routes = []
        for item in pendaki_routes.split(","):
            item = item.strip()
            if not item:
                continue
            if ":" not in item:
                raise RuntimeError(
                    "Format pendaki_routes harus seperti a:5,b:4."
                )
            raw_route, raw_count = item.split(":", 1)
            route_name = _resolve_route(raw_route, active_trails)
            count = _positive_int(raw_count.strip(), f"Jumlah pendaki untuk {raw_route}")
            routes.extend([route_name] * count)
    else:
        count = _positive_int(_cfg(context, "pendaki").strip(), "pendaki")
        raw_route = _cfg(context, "jalur").strip() or _cfg(context, "trail_name").strip()
        route_name = _resolve_route(raw_route, active_trails)
        routes = [route_name] * count

    total = len(routes)
    if total < 1:
        raise RuntimeError("Minimal harus ada 1 pendaki.")
    if total > 10:
        raise RuntimeError(f"Total pendaki tidak boleh lebih dari 10; diminta {total}.")
    return routes


def _load_yaml(path: str):
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("PyYAML belum tersedia, tidak bisa membaca hikers_file.") from exc

    try:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
    except OSError as exc:
        raise RuntimeError(f"Tidak bisa membaca hikers_file: {path}") from exc

    if not isinstance(data, dict):
        raise RuntimeError("hikers_file harus berisi YAML mapping/object.")
    return data


def _float_pair(raw_value, label: str):
    if not isinstance(raw_value, (list, tuple)) or len(raw_value) != 2:
        raise RuntimeError(f"{label} harus berisi dua angka, contoh: [-5.0, 5.0].")
    try:
        return float(raw_value[0]), float(raw_value[1])
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"{label} harus berisi angka.") from exc


def _bool_value(value, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _hiker_routes_from_yaml(data, active_trails):
    hikers = data.get("hikers", {})
    if not isinstance(hikers, dict):
        raise RuntimeError("Bagian 'hikers' di hikers_file harus berupa object.")

    routes = []
    route_counts = hikers.get("routes") or hikers.get("route_counts")
    groups = hikers.get("groups")

    if isinstance(route_counts, dict):
        for raw_route, raw_count in route_counts.items():
            route_name = _resolve_route(str(raw_route), active_trails)
            count = _positive_int(str(raw_count), f"Jumlah pendaki untuk {raw_route}")
            routes.extend([route_name] * count)
    elif isinstance(groups, list):
        for group in groups:
            if not isinstance(group, dict):
                raise RuntimeError("Setiap item hikers.groups harus berupa object.")
            route_name = _resolve_route(str(group.get("route", "")), active_trails)
            count = _positive_int(str(group.get("count", 1)), f"Jumlah pendaki untuk {route_name}")
            routes.extend([route_name] * count)
    else:
        raise RuntimeError(
            "hikers_file harus punya hikers.routes, contoh: hikers: {routes: {a: 5, b: 4}}"
        )

    total = len(routes)
    if total < 1:
        raise RuntimeError("Minimal harus ada 1 pendaki di hikers_file.")
    if total > 10:
        raise RuntimeError(f"Total pendaki tidak boleh lebih dari 10; hikers_file meminta {total}.")
    return routes


def _relay_station(spawn_cfg, rng, lora_nodes):
    relay_name = spawn_cfg.get("relay_name") or spawn_cfg.get("relay")
    if relay_name:
        relay_name = str(relay_name)
        for station in lora_nodes:
            if station.name == relay_name:
                return station
        choices = ", ".join(station.name for station in lora_nodes)
        raise RuntimeError(f"relay_name '{relay_name}' tidak ditemukan. Pilihan: {choices}")
    return rng.choice(list(lora_nodes))


def _random_highland_position(rng, spawn_cfg):
    x_min, x_max = _float_pair(spawn_cfg.get("x_range", [28.0, 96.0]), "spawn.x_range")
    y_min, y_max = _float_pair(spawn_cfg.get("y_range", [28.0, 88.0]), "spawn.y_range")
    min_terrain_z = float(spawn_cfg.get("min_terrain_z", 24.0))

    best_x = best_y = 0.0
    best_z = -1.0
    for _ in range(500):
        x = rng.uniform(x_min, x_max)
        y = rng.uniform(y_min, y_max)
        z = terrain_height_world(x, y)
        if z >= min_terrain_z:
            return x, y
        if z > best_z:
            best_x, best_y, best_z = x, y, z
    return best_x, best_y


def _random_position(rng, spawn_cfg, active_trails, first_route: str, route_name: str, lora_nodes, base_station):
    mode = str(spawn_cfg.get("mode", "random_box")).strip().lower()
    if mode in {"random_relay", "relay"}:
        station = _relay_station(spawn_cfg, rng, lora_nodes)
        x_min, x_max = _float_pair(spawn_cfg.get("x_offset_range", [-2.0, 2.0]), "spawn.x_offset_range")
        y_min, y_max = _float_pair(spawn_cfg.get("y_offset_range", [-2.0, 2.0]), "spawn.y_offset_range")
        return station.x + rng.uniform(x_min, x_max), station.y + rng.uniform(y_min, y_max)

    if mode in {"random_highland", "highland", "random_top", "top"}:
        return _random_highland_position(rng, spawn_cfg)

    if "x_range" in spawn_cfg or "y_range" in spawn_cfg:
        x_min, x_max = _float_pair(spawn_cfg.get("x_range"), "spawn.x_range")
        y_min, y_max = _float_pair(spawn_cfg.get("y_range"), "spawn.y_range")
        return rng.uniform(x_min, x_max), rng.uniform(y_min, y_max)

    center = spawn_cfg.get("center", "base_start")
    if isinstance(center, (list, tuple)):
        center_x, center_y = _float_pair(center, "spawn.center")
    else:
        center_name = str(center).strip().lower()
        if center_name == "route_start":
            center_x, center_y = active_trails[route_name][0]
        elif center_name == "base_start":
            center_x, center_y = active_trails[first_route][0]
        elif center_name in {"base_station", "base"}:
            center_x, center_y = base_station.x, base_station.y
        elif center_name in {"relay", "random_relay"}:
            station = _relay_station(spawn_cfg, rng, lora_nodes)
            center_x, center_y = station.x, station.y
        else:
            station_lookup = {station.name: station for station in lora_nodes}
            if center_name in station_lookup:
                station = station_lookup[center_name]
                center_x, center_y = station.x, station.y
            else:
                center_route = _resolve_route(center_name, active_trails)
                center_x, center_y = active_trails[center_route][0]

    x_min, x_max = _float_pair(spawn_cfg.get("x_offset_range", [-3.0, 3.0]), "spawn.x_offset_range")
    y_min, y_max = _float_pair(spawn_cfg.get("y_offset_range", [-3.0, 3.0]), "spawn.y_offset_range")
    return center_x + rng.uniform(x_min, x_max), center_y + rng.uniform(y_min, y_max)


def _hiker_specs_from_yaml(path: str, active_trails, lora_nodes, base_station):
    data = _load_yaml(path)
    routes = _hiker_routes_from_yaml(data, active_trails)
    spawn_cfg = data.get("spawn", {})
    if not isinstance(spawn_cfg, dict):
        raise RuntimeError("Bagian 'spawn' di hikers_file harus berupa object.")

    mode = str(spawn_cfg.get("mode", "random_box")).strip().lower()
    valid_modes = {
        "random_box",
        "random",
        "random_relay",
        "relay",
        "random_highland",
        "highland",
        "random_top",
        "top",
    }
    if mode not in valid_modes:
        raise RuntimeError(
            "spawn.mode harus random_box, random_relay, atau random_highland."
        )

    seed = int(spawn_cfg.get("seed", data.get("hikers", {}).get("random_seed", 42)))
    min_spacing = max(0.0, float(spawn_cfg.get("min_spacing", 0.8)))
    snap_to_path = _bool_value(spawn_cfg.get("snap_to_path"), True)
    rng = random.Random(seed)
    first_route = routes[0]
    positions = []
    specs = []

    for route_name in routes:
        x = y = 0.0
        for attempt in range(200):
            x, y = _random_position(
                rng, spawn_cfg, active_trails, first_route, route_name, lora_nodes, base_station
            )
            if all(hypot(x - px, y - py) >= min_spacing for px, py in positions):
                break
            if attempt == 199:
                break
        if snap_to_path:
            route_points = active_trails[route_name]
            start_distance = nearest_trail_progress(x, y, route_points) * trail_length(route_points)
            x, y, _ = point_on_trail(start_distance, route_points)
        positions.append((x, y))
        specs.append(
            {
                "route": route_name,
                "spawn_x": x,
                "spawn_y": y,
                "offset_x": 0.0,
                "offset_y": 0.0,
            }
        )

    return specs


def _formation_offsets(total: int, spacing_col: float, spacing_row: float, jitter: float, seed: int):
    if total == 1:
        return [(0.0, 0.0)]

    rng = random.Random(seed)
    rows = min(2, total)
    cols = ceil(total / rows)
    jitter = max(0.0, jitter)
    offsets = []
    for index in range(total):
        row = index % rows
        col = index // rows
        x = (col - (cols - 1) / 2.0) * spacing_col
        y = (row - (rows - 1) / 2.0) * spacing_row
        x += rng.uniform(-jitter, jitter)
        y += rng.uniform(-jitter, jitter)
        offsets.append((x, y))
    return offsets


def _custom_spawn_xy(raw_xy: str):
    parts = [part.strip() for part in raw_xy.split(",")]
    if len(parts) != 2:
        raise RuntimeError('spawn_xy harus memakai format "x,y", contoh spawn_xy:="-104,-94".')
    try:
        return float(parts[0]), float(parts[1])
    except ValueError as exc:
        raise RuntimeError('spawn_xy harus angka, contoh spawn_xy:="-104,-94".') from exc


def _parse_spreading_factors(context, n_hikers: int) -> list:
    """
    Parse spreading_factors (comma-separated) → list SF per hiker (round-robin).
    Jika kosong, semua hiker pakai spreading_factor tunggal.
    Valid SF: 7, 8, 9, 10, 11, 12 (SX1276/LLCC68 BW=125 kHz).
    Contoh: spreading_factors:="7,9,12" dengan 3 pendaki → hiker1=SF7, hiker2=SF9, hiker3=SF12
    """
    raw = _cfg(context, "spreading_factors").strip()
    base_sf = int(_cfg(context, "spreading_factor"))
    if not raw:
        return [base_sf] * n_hikers
    parts = [p.strip() for p in raw.split(",") if p.strip()]
    valid_sfs = {7, 8, 9, 10, 11, 12}
    sfs = []
    for p in parts:
        try:
            sf = int(p)
        except ValueError as exc:
            raise RuntimeError(f"spreading_factors harus integer, bukan '{p}'") from exc
        if sf not in valid_sfs:
            raise RuntimeError(f"SF tidak valid: {sf}. Pilihan: {sorted(valid_sfs)}")
        sfs.append(sf)
    return [sfs[i % len(sfs)] for i in range(n_hikers)]


def _launch_setup(context, *args, **kwargs):
    routes_file = _cfg(context, "routes_file").strip()
    active_trails = _active_trails(routes_file)
    lora_nodes, base_station = _scenario_stations(routes_file)
    hikers_file = _cfg(context, "hikers_file").strip()

    if hikers_file:
        hiker_specs = _hiker_specs_from_yaml(hikers_file, active_trails, lora_nodes, base_station)
    else:
        hiker_routes = _parse_hiker_routes(context, active_trails)
        spawn_source = _cfg(context, "spawn_source").strip().lower() or "base_start"
        if spawn_source not in {"base_start", "route_start", "custom"}:
            raise RuntimeError("spawn_source harus base_start, route_start, atau custom.")

        spacing_col = float(_cfg(context, "spawn_spacing_col"))
        spacing_row = float(_cfg(context, "spawn_spacing_row"))
        jitter = float(_cfg(context, "spawn_jitter"))
        spawn_seed = int(_cfg(context, "spawn_seed"))
        offsets = _formation_offsets(len(hiker_routes), spacing_col, spacing_row, jitter, spawn_seed)

        first_route_for_anchor = hiker_routes[0]
        base_anchor = active_trails[first_route_for_anchor][0]
        custom_anchor = None
        if spawn_source == "custom":
            custom_anchor = _custom_spawn_xy(_cfg(context, "spawn_xy").strip())

        hiker_specs = []
        for index, route_name in enumerate(hiker_routes):
            offset_x, offset_y = offsets[index]
            if spawn_source == "custom":
                anchor_x, anchor_y = custom_anchor
            elif spawn_source == "route_start":
                anchor_x, anchor_y = active_trails[route_name][0]
            else:
                anchor_x, anchor_y = base_anchor
            hiker_specs.append(
                {
                    "route": route_name,
                    "spawn_x": anchor_x,
                    "spawn_y": anchor_y,
                    "offset_x": offset_x,
                    "offset_y": offset_y,
                }
            )

    total_hikers = len(hiker_specs)
    multi_mode = total_hikers > 1
    sf_list = _parse_spreading_factors(context, total_hikers)

    pkg_share = FindPackageShare("hiking_lora_sim").perform(context)
    first_route = hiker_specs[0]["route"]
    world_route = first_route if first_route in _BUILTIN_ROUTES else "ridge_route"
    world_path = os.path.join(pkg_share, "worlds", f"hiking_mountain_{world_route}.sdf")
    ros_gz_sim_launch = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )
    use_gazebo = LaunchConfiguration("use_gazebo")

    common_params = {
        "meters_per_world_unit": LaunchConfiguration("meters_per_world_unit"),
        "reference_lat": LaunchConfiguration("reference_lat"),
        "reference_lon": LaunchConfiguration("reference_lon"),
    }

    actions = []
    if _bool_value(_cfg(context, "use_gazebo"), True) and _bool_value(
        _cfg(context, "use_gazebo_gui_keyboard"), True
    ):
        try:
            gui_plugin_prefix = get_package_prefix("hiking_lora_gz_gui")
        except PackageNotFoundError as exc:
            raise RuntimeError(
                "Gazebo GUI keyboard plugin belum ter-build. Jalankan: "
                "colcon build --symlink-install"
            ) from exc
        actions.append(
            SetEnvironmentVariable(
                "GZ_GUI_PLUGIN_PATH",
                [
                    os.path.join(gui_plugin_prefix, "lib"),
                    ":",
                    EnvironmentVariable("GZ_GUI_PLUGIN_PATH", default_value=""),
                ],
            )
        )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ros_gz_sim_launch),
            condition=IfCondition(use_gazebo),
            launch_arguments={
                "gz_args": f"{_cfg(context, 'gz_args')} {world_path}",
            }.items(),
        )
    )

    spawn_seed = int(_cfg(context, "spawn_seed"))
    for index, spec in enumerate(hiker_specs, start=1):
        route_name = spec["route"]
        hiker_id = f"hiker_{index}" if multi_mode else "hiker"
        topic_prefix = f"/{hiker_id}" if multi_mode else ""
        namespace = hiker_id if multi_mode else ""
        model_name = hiker_id if multi_mode else "hiker"
        anchor_x = float(spec["spawn_x"])
        anchor_y = float(spec["spawn_y"])
        offset_x = float(spec["offset_x"])
        offset_y = float(spec["offset_y"])

        actions.append(
            Node(
                package="hiking_lora_sim",
                executable="hiker_agent",
                namespace=namespace,
                output="screen",
                parameters=[
                    common_params,
                    {
                        "hiker_id": hiker_id,
                        "topic_prefix": topic_prefix,
                        "tx_power_dbm": LaunchConfiguration("tx_power_dbm"),
                        "gps_noise_std_m": LaunchConfiguration("gps_noise_std_m"),
                        "ttff_delay_s": LaunchConfiguration("ttff_delay_s"),
                        "speed_world_units_s": LaunchConfiguration("hiker_speed_world_units_s"),
                        "trail_name": route_name,
                        "gazebo_pose_control": use_gazebo,
                        "gazebo_visual_rate_hz": LaunchConfiguration("gazebo_visual_rate_hz"),
                        "gazebo_hiker_z_offset": LaunchConfiguration("gazebo_hiker_z_offset"),
                        "gazebo_hiker_model": model_name,
                        "spawn_model": use_gazebo,
                        "spawn_x": f"{anchor_x:.6f}",
                        "spawn_y": f"{anchor_y:.6f}",
                        "spawn_offset_x": offset_x,
                        "spawn_offset_y": offset_y,
                        "random_seed": spawn_seed + index * 9973,
                        "publish_aggregate_topics": multi_mode,
                        "battery_capacity_mah": LaunchConfiguration("battery_capacity_mah"),
                        "tx_current_ma": LaunchConfiguration("tx_current_ma"),
                        "idle_current_ma": LaunchConfiguration("idle_current_ma"),
                        "routes_file": LaunchConfiguration("routes_file"),
                        "weather": LaunchConfiguration("weather"),
                        "temperature_c": LaunchConfiguration("temperature_c"),
                        "humidity_pct": LaunchConfiguration("humidity_pct"),
                        "manual_move_speed_world_units_s": LaunchConfiguration("manual_move_speed_world_units_s"),
                        "manual_turn_rate_rad_s": LaunchConfiguration("manual_turn_rate_rad_s"),
                    },
                ],
            )
        )

        actions.append(
            Node(
                package="hiking_lora_sim",
                executable="lora_network",
                namespace=namespace,
                output="screen",
                parameters=[
                    common_params,
                    {
                        "hiker_id": hiker_id,
                        "topic_prefix": topic_prefix,
                        "publish_global_events": multi_mode,
                        "active_hiker_count": total_hikers,
                        "tx_power_dbm": LaunchConfiguration("tx_power_dbm"),
                        "antenna_gain_db": LaunchConfiguration("antenna_gain_db"),
                        "terrain_loss_db_per_km": LaunchConfiguration("terrain_loss_db_per_km"),
                        # Per-hiker SF: integer langsung (bukan str) agar tipe INTEGER match
                        "spreading_factor": sf_list[index - 1],
                        "fading_model": LaunchConfiguration("fading_model"),
                        "rician_k_db": LaunchConfiguration("rician_k_db"),
                        "routes_file": LaunchConfiguration("routes_file"),
                        "weather": LaunchConfiguration("weather"),
                        "temperature_c": LaunchConfiguration("temperature_c"),
                        "humidity_pct": LaunchConfiguration("humidity_pct"),
                    },
                ],
            )
        )

    if _bool_value(_cfg(context, "use_camera_views"), True) and _bool_value(
        _cfg(context, "use_gazebo"), True
    ):
        camera_target = _cfg(context, "camera_target_hiker").strip()
        if not camera_target:
            camera_target = "hiker_1" if multi_mode else "hiker"
        actions.append(
            Node(
                package="hiking_lora_sim",
                executable="camera_controller",
                output="screen",
                parameters=[
                    {
                        "target_hiker_id": camera_target,
                        "active_hiker_count": total_hikers,
                        "camera_update_rate_hz": LaunchConfiguration("camera_update_rate_hz"),
                        "gui_camera_timeout_ms": LaunchConfiguration("gui_camera_timeout_ms"),
                        "follow_camera_height_world_units": LaunchConfiguration(
                            "follow_camera_height_world_units"
                        ),
                        "follow_camera_offset_x": LaunchConfiguration("follow_camera_offset_x"),
                        "follow_camera_offset_y": LaunchConfiguration("follow_camera_offset_y"),
                        "follow_camera_pitch_rad": LaunchConfiguration("follow_camera_pitch_rad"),
                        "follow_camera_roll_rad": LaunchConfiguration("follow_camera_roll_rad"),
                        "follow_camera_yaw_mode": LaunchConfiguration("follow_camera_yaw_mode"),
                        "follow_camera_fixed_yaw_rad": LaunchConfiguration(
                            "follow_camera_fixed_yaw_rad"
                        ),
                        "overview_camera_x": LaunchConfiguration("overview_camera_x"),
                        "overview_camera_y": LaunchConfiguration("overview_camera_y"),
                        "overview_camera_z": LaunchConfiguration("overview_camera_z"),
                        "overview_camera_roll_rad": LaunchConfiguration("overview_camera_roll_rad"),
                        "overview_camera_pitch_rad": LaunchConfiguration("overview_camera_pitch_rad"),
                        "overview_camera_yaw_rad": LaunchConfiguration("overview_camera_yaw_rad"),
                        "overview_camera_min_z": LaunchConfiguration("overview_camera_min_z"),
                        "overview_camera_max_z": LaunchConfiguration("overview_camera_max_z"),
                    }
                ],
            )
        )

    actions.extend(
        [
            Node(
                package="hiking_lora_sim",
                executable="base_station_display",
                output="screen",
            ),
            Node(
                package="hiking_lora_sim",
                executable="keyboard_teleop",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(LaunchConfiguration("use_keyboard_teleop")),
                parameters=[
                    {
                        "overview_camera_xy_step": LaunchConfiguration("overview_camera_xy_step"),
                        "overview_camera_z_step": LaunchConfiguration("overview_camera_z_step"),
                        "overview_camera_angle_step_rad": LaunchConfiguration(
                            "overview_camera_angle_step_rad"
                        ),
                    }
                ],
            ),
            Node(
                package="hiking_lora_sim",
                executable="dashboard",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(LaunchConfiguration("use_dashboard")),
                parameters=[
                    {"refresh_rate_hz": LaunchConfiguration("dashboard_refresh_hz")},
                ],
            ),
            Node(
                package="hiking_lora_sim",
                executable="web_dashboard",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_web_dashboard")),
                parameters=[
                    {
                        "host": LaunchConfiguration("web_dashboard_host"),
                        "port": LaunchConfiguration("web_dashboard_port"),
                        "routes_file": LaunchConfiguration("routes_file"),
                    }
                ],
            ),
        ]
    )
    return actions


def generate_launch_description():
    pkg_share = FindPackageShare("hiking_lora_sim")

    return LaunchDescription(
        [
            # --- Gazebo ---
            DeclareLaunchArgument("use_gazebo", default_value="true"),
            DeclareLaunchArgument("gz_args", default_value="-r -v 3 --render-engine-gui ogre2"),
            DeclareLaunchArgument("use_gazebo_gui_keyboard", default_value="true"),

            # --- GPS & skala ---
            DeclareLaunchArgument("meters_per_world_unit", default_value="35.0"),
            DeclareLaunchArgument("reference_lat", default_value="-6.89148"),
            DeclareLaunchArgument("reference_lon", default_value="107.61066"),

            # --- Hiker ---
            DeclareLaunchArgument("gps_noise_std_m", default_value="2.0"),
            DeclareLaunchArgument("ttff_delay_s", default_value="8.0"),
            DeclareLaunchArgument(
                "trail_name",
                default_value="ridge_route",
                choices=["ridge_route", "valley_route", "crater_route"],
            ),
            DeclareLaunchArgument("jalur", default_value=""),
            DeclareLaunchArgument("pendaki", default_value="1"),
            DeclareLaunchArgument("pendaki_routes", default_value=""),
            DeclareLaunchArgument("hikers_file", default_value=""),
            DeclareLaunchArgument(
                "spawn_source",
                default_value="base_start",
                choices=["base_start", "route_start", "custom"],
            ),
            DeclareLaunchArgument("spawn_xy", default_value=""),
            DeclareLaunchArgument("spawn_spacing_col", default_value="1.2"),
            DeclareLaunchArgument("spawn_spacing_row", default_value="1.0"),
            DeclareLaunchArgument("spawn_jitter", default_value="0.25"),
            DeclareLaunchArgument("spawn_seed", default_value="42"),
            DeclareLaunchArgument("hiker_speed_world_units_s", default_value="0.85"),
            DeclareLaunchArgument("gazebo_visual_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("gazebo_hiker_z_offset", default_value="0.08"),
            DeclareLaunchArgument("use_keyboard_teleop", default_value="false"),
            DeclareLaunchArgument("manual_move_speed_world_units_s", default_value="0.85"),
            DeclareLaunchArgument("manual_turn_rate_rad_s", default_value="1.8"),

            # --- Kamera Gazebo ---
            DeclareLaunchArgument("use_camera_views", default_value="true"),
            DeclareLaunchArgument("camera_target_hiker", default_value=""),
            DeclareLaunchArgument("camera_update_rate_hz", default_value="8.0"),
            DeclareLaunchArgument("gui_camera_timeout_ms", default_value="80"),
            DeclareLaunchArgument("follow_camera_height_world_units", default_value="24.0"),
            DeclareLaunchArgument("follow_camera_offset_x", default_value="0.0"),
            DeclareLaunchArgument("follow_camera_offset_y", default_value="0.0"),
            DeclareLaunchArgument("follow_camera_roll_rad", default_value="0.0"),
            DeclareLaunchArgument("follow_camera_pitch_rad", default_value="1.5708"),
            DeclareLaunchArgument(
                "follow_camera_yaw_mode",
                default_value="fixed",
                choices=["hiker", "fixed"],
            ),
            DeclareLaunchArgument("follow_camera_fixed_yaw_rad", default_value="0.0"),
            DeclareLaunchArgument("overview_camera_x", default_value="0.0"),
            DeclareLaunchArgument("overview_camera_y", default_value="0.0"),
            DeclareLaunchArgument("overview_camera_z", default_value="220.0"),
            DeclareLaunchArgument("overview_camera_roll_rad", default_value="0.0"),
            DeclareLaunchArgument("overview_camera_pitch_rad", default_value="1.5708"),
            DeclareLaunchArgument("overview_camera_yaw_rad", default_value="0.0"),
            DeclareLaunchArgument("overview_camera_min_z", default_value="20.0"),
            DeclareLaunchArgument("overview_camera_max_z", default_value="420.0"),
            DeclareLaunchArgument("overview_camera_xy_step", default_value="8.0"),
            DeclareLaunchArgument("overview_camera_z_step", default_value="10.0"),
            DeclareLaunchArgument("overview_camera_angle_step_rad", default_value="0.10"),

            # --- LoRa RF ---
            # EByte E220-900T22D: TX max 22 dBm (Ref: EByte datasheet v1.0)
            DeclareLaunchArgument("tx_power_dbm", default_value="22.0"),
            # Antena SMA 3 dBi (hardware proyek)
            DeclareLaunchArgument("antenna_gain_db", default_value="5.0"),
            # Terrain scatter (ITU-R P.452 rural sub-GHz: 0.5–1.5 dB/km)
            DeclareLaunchArgument("terrain_loss_db_per_km", default_value="1.0"),
            # SF tunggal (default). Gunakan spreading_factors untuk multi-SF per hiker.
            DeclareLaunchArgument("spreading_factor", default_value="9"),
            # SF per hiker (comma-separated, round-robin). Contoh: "7,9,12"
            # Jika kosong, semua hiker pakai spreading_factor di atas.
            DeclareLaunchArgument("spreading_factors", default_value=""),
            DeclareLaunchArgument("fading_model", default_value="rayleigh"),
            DeclareLaunchArgument("rician_k_db", default_value="10.0"),
            # Cuaca & lingkungan
            DeclareLaunchArgument(
                "weather",
                default_value="clear",
                choices=["clear", "fog", "light_rain", "heavy_rain", "thunderstorm"],
            ),
            DeclareLaunchArgument("temperature_c", default_value="22.0"),
            DeclareLaunchArgument("humidity_pct", default_value="60.0"),

            # --- Fitur 8: Baterai ---
            DeclareLaunchArgument("battery_capacity_mah", default_value="3000.0"),
            DeclareLaunchArgument("tx_current_ma", default_value="120.0"),
            DeclareLaunchArgument("idle_current_ma", default_value="3.0"),

            # --- Fitur 9: Rute dinamis ---
            DeclareLaunchArgument("routes_file", default_value=""),

            # --- Fitur 6: Dashboard ---
            DeclareLaunchArgument("use_dashboard", default_value="false"),
            DeclareLaunchArgument("dashboard_refresh_hz", default_value="0.5"),
            DeclareLaunchArgument("use_web_dashboard", default_value="false"),
            DeclareLaunchArgument("web_dashboard_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("web_dashboard_port", default_value="8080"),

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
            SetEnvironmentVariable(
                "SDF_PATH",
                [
                    PathJoinSubstitution([pkg_share, "models"]),
                    ":",
                    EnvironmentVariable("SDF_PATH", default_value=""),
                ],
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
