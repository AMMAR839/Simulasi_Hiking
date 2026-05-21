from dataclasses import dataclass
from math import atan2, cos, exp, hypot, pi, sin, sqrt
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


DEFAULT_METERS_PER_WORLD_UNIT = 35.0
WORLD_EXTENT = 120.0
ACTIVE_TRAIL_NAME = "ridge_route"

TRAILS: Dict[str, List[Tuple[float, float]]] = {
    "ridge_route": [
        (-104.0, -94.0),
        (-92.0, -78.0),
        (-77.0, -62.0),
        (-61.0, -48.0),
        (-47.0, -34.0),
        (-35.0, -18.0),
        (-18.0, -10.0),
        (2.0, -4.0),
        (22.0, 9.0),
        (39.0, 28.0),
        (54.0, 46.0),
        (71.0, 63.0),
        (91.0, 81.0),
    ],
    "valley_route": [
        (-104.0, -94.0),
        (-96.0, -67.0),
        (-83.0, -43.0),
        (-68.0, -20.0),
        (-50.0, 1.0),
        (-28.0, 19.0),
        (-5.0, 31.0),
        (19.0, 42.0),
        (43.0, 51.0),
        (68.0, 64.0),
        (91.0, 81.0),
    ],
    "crater_route": [
        (-104.0, -94.0),
        (-82.0, -86.0),
        (-58.0, -72.0),
        (-38.0, -52.0),
        (-16.0, -33.0),
        (10.0, -19.0),
        (31.0, 1.0),
        (46.0, 24.0),
        (54.0, 45.0),
        (42.0, 63.0),
        (63.0, 77.0),
        (91.0, 81.0),
    ],
}

TRAIL_POINTS: List[Tuple[float, float]] = TRAILS[ACTIVE_TRAIL_NAME]


@dataclass(frozen=True)
class Station:
    name: str
    x: float
    y: float
    z: float
    kind: str


@dataclass(frozen=True)
class RadioObstacle:
    name: str
    x: float
    y: float
    radius: float
    loss_db: float
    kind: str


BASE_STATION = Station("base_station", -106.0, -96.0, 0.0, "base")

LORA_NODES: List[Station] = [
    Station("node_basecamp_gate", -91.0, -78.0, 0.0, "relay"),
    Station("node_valley_watch", -66.0, -23.0, 0.0, "relay"),
    Station("node_forest_pass", -38.0, -23.0, 0.0, "relay"),
    Station("node_ridge_mid", 4.0, -3.0, 0.0, "relay"),
    Station("node_crater_edge", 54.0, 45.0, 0.0, "relay"),
    Station("node_north_saddle", 70.0, 64.0, 0.0, "relay"),
    Station("node_summit_view", 91.0, 81.0, 0.0, "relay"),
]

RADIO_OBSTACLES: List[RadioObstacle] = [
    RadioObstacle("lower_dense_forest", -55.0, -46.0, 22.0, 14.0, "trees"),
    RadioObstacle("valley_forest", -38.0, 5.0, 20.0, 11.0, "trees"),
    RadioObstacle("rocky_cliff_band", 20.0, 16.0, 18.0, 9.0, "rocks"),
    RadioObstacle("crater_rim", 54.0, 45.0, 16.0, 10.0, "crater"),
    RadioObstacle("summit_shadow_ridge", 76.0, 70.0, 18.0, 8.0, "terrain"),
]


def trail_length(points: Sequence[Tuple[float, float]] = TRAIL_POINTS) -> float:
    return sum(
        hypot(points[index + 1][0] - points[index][0], points[index + 1][1] - points[index][1])
        for index in range(len(points) - 1)
    )


def point_on_trail(
    distance_world: float,
    points: Sequence[Tuple[float, float]] = TRAIL_POINTS,
) -> Tuple[float, float, float]:
    """Return x, y, yaw along the selected hiker trail."""
    total = trail_length(points)
    if total <= 0.0:
        x, y = points[0]
        return x, y, 0.0

    remaining = min(distance_world, total)
    for index in range(len(points) - 1):
        x0, y0 = points[index]
        x1, y1 = points[index + 1]
        segment = hypot(x1 - x0, y1 - y0)
        if remaining <= segment:
            ratio = 0.0 if segment == 0.0 else remaining / segment
            x = x0 + (x1 - x0) * ratio
            y = y0 + (y1 - y0) * ratio
            return x, y, atan2(y1 - y0, x1 - x0)
        remaining -= segment

    x0, y0 = points[-2]
    x1, y1 = points[-1]
    return x1, y1, atan2(y1 - y0, x1 - x0)


def terrain_height_world(x: float, y: float) -> float:
    """Procedural mountain height in Gazebo world units."""
    north_climb = 0.055 * (y + WORLD_EXTENT)
    summit = 25.0 * gaussian2d(x, y, 74.0, 70.0, 42.0, 34.0)
    east_peak = 12.0 * gaussian2d(x, y, 38.0, 38.0, 34.0, 42.0)
    west_ridge = 10.0 * ridge_gaussian(x, y, -0.35, -32.0, 15.0, -45.0, 80.0)
    main_ridge = 12.0 * ridge_gaussian(x, y, 0.55, 15.0, 13.0, 25.0, 92.0)
    valley_cut = 9.5 * gaussian2d(x, y, -58.0, -4.0, 30.0, 45.0)
    south_valley = 4.5 * gaussian2d(x, y, -78.0, -75.0, 24.0, 22.0)

    crater_distance = hypot(x - 54.0, y - 45.0)
    crater_rim = 4.6 * exp(-((crater_distance - 13.0) ** 2) / (2.0 * 3.4**2))
    crater_bowl = 5.2 * exp(-(crater_distance**2) / (2.0 * 8.0**2))

    roughness = (
        1.1 * sin(x * 0.115)
        + 0.9 * cos(y * 0.092)
        + 0.7 * sin((x + y) * 0.064)
        + 0.45 * cos((x - 2.0 * y) * 0.051)
    )

    height = (
        1.2
        + north_climb
        + summit
        + east_peak
        + west_ridge
        + main_ridge
        + crater_rim
        - crater_bowl
        - valley_cut
        - south_valley
        + roughness
    )
    return max(0.35, height)


def terrain_altitude_m(x: float, y: float, meters_per_world_unit: float) -> float:
    """Approximate mountain altitude used by the GPS and radio model."""
    return 920.0 + terrain_height_world(x, y) * meters_per_world_unit * 0.82


def nearest_trail_progress(
    x: float,
    y: float,
    points: Sequence[Tuple[float, float]] = TRAIL_POINTS,
) -> float:
    total = trail_length(points)
    if total <= 0.0:
        return 0.0

    best_distance = float("inf")
    best_progress = 0.0
    traveled = 0.0

    for index in range(len(points) - 1):
        ax, ay = points[index]
        bx, by = points[index + 1]
        dx = bx - ax
        dy = by - ay
        segment_len_sq = dx * dx + dy * dy
        if segment_len_sq == 0.0:
            continue
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / segment_len_sq))
        px = ax + t * dx
        py = ay + t * dy
        distance = hypot(x - px, y - py)
        if distance < best_distance:
            best_distance = distance
            best_progress = (traveled + hypot(dx, dy) * t) / total
        traveled += hypot(dx, dy)

    return max(0.0, min(1.0, best_progress))


def local_to_gps(
    x: float,
    y: float,
    altitude_m: float,
    reference_lat: float,
    reference_lon: float,
    meters_per_world_unit: float,
) -> Tuple[float, float, float]:
    north_m = y * meters_per_world_unit
    east_m = x * meters_per_world_unit
    lat = reference_lat + north_m / 111_320.0
    lon = reference_lon + east_m / (111_320.0 * cos(reference_lat * pi / 180.0))
    return lat, lon, altitude_m


def all_radio_stations() -> Iterable[Station]:
    yield BASE_STATION
    yield from LORA_NODES


def gaussian2d(x: float, y: float, cx: float, cy: float, sx: float, sy: float) -> float:
    return exp(-(((x - cx) ** 2) / (2.0 * sx * sx) + ((y - cy) ** 2) / (2.0 * sy * sy)))


def ridge_gaussian(
    x: float,
    y: float,
    slope: float,
    intercept: float,
    width: float,
    center_x: float,
    length: float,
) -> float:
    distance_to_line = abs(y - slope * x - intercept) / sqrt(slope * slope + 1.0)
    length_falloff = exp(-(((x - center_x) ** 2) / (2.0 * length * length)))
    return exp(-((distance_to_line**2) / (2.0 * width * width))) * length_falloff


def station_radio_altitude_m(
    station: Station,
    meters_per_world_unit: float = DEFAULT_METERS_PER_WORLD_UNIT,
) -> float:
    mast_height_m = 14.0 if station.kind == "base" else 10.0
    return terrain_altitude_m(station.x, station.y, meters_per_world_unit) + mast_height_m


def load_scenario_yaml(path: str) -> Optional[Dict[str, Any]]:
    """Muat konfigurasi skenario dari file YAML.

    Mengembalikan dict dengan kunci 'trails', 'lora_nodes', 'base_station',
    'radio_obstacles', atau None jika file tidak ditemukan / tidak valid.
    """
    try:
        import yaml
    except ImportError:
        return None

    try:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
    except (FileNotFoundError, OSError, yaml.YAMLError):
        return None

    if not isinstance(data, dict):
        return None

    result: Dict[str, Any] = {}

    # Muat trails
    raw_trails = data.get("trails")
    if isinstance(raw_trails, dict):
        trails: Dict[str, List[Tuple[float, float]]] = {}
        for name, trail_data in raw_trails.items():
            waypoints = trail_data.get("waypoints") if isinstance(trail_data, dict) else trail_data
            if isinstance(waypoints, list) and len(waypoints) >= 2:
                trails[name] = [
                    (float(wp[0]), float(wp[1]))
                    for wp in waypoints
                    if isinstance(wp, (list, tuple)) and len(wp) >= 2
                ]
        if trails:
            result["trails"] = trails

    # Muat lora_nodes
    raw_nodes = data.get("lora_nodes")
    if isinstance(raw_nodes, list):
        nodes: List[Station] = []
        for node in raw_nodes:
            if isinstance(node, dict) and "name" in node:
                nodes.append(Station(
                    name=str(node["name"]),
                    x=float(node.get("x", 0.0)),
                    y=float(node.get("y", 0.0)),
                    z=float(node.get("z", 0.0)),
                    kind=str(node.get("kind", "relay")),
                ))
        if nodes:
            result["lora_nodes"] = nodes

    # Muat base_station
    raw_base = data.get("base_station")
    if isinstance(raw_base, dict) and "x" in raw_base:
        result["base_station"] = Station(
            name=str(raw_base.get("name", "base_station")),
            x=float(raw_base["x"]),
            y=float(raw_base.get("y", 0.0)),
            z=float(raw_base.get("z", 0.0)),
            kind=str(raw_base.get("kind", "base")),
        )

    # Muat radio_obstacles
    raw_obs = data.get("radio_obstacles")
    if isinstance(raw_obs, list):
        obstacles: List[RadioObstacle] = []
        for obs in raw_obs:
            if isinstance(obs, dict) and "name" in obs:
                obstacles.append(RadioObstacle(
                    name=str(obs["name"]),
                    x=float(obs.get("x", 0.0)),
                    y=float(obs.get("y", 0.0)),
                    radius=float(obs.get("radius", 10.0)),
                    loss_db=float(obs.get("loss_db", 10.0)),
                    kind=str(obs.get("kind", "terrain")),
                ))
        result["radio_obstacles"] = obstacles

    return result if result else None
