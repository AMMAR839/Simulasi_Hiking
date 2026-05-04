#!/usr/bin/env python3
import math
import os
import random
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent
sys.path.insert(0, str(PACKAGE_DIR))

from hiking_lora_sim.scenario import (  # noqa: E402
    BASE_STATION,
    LORA_NODES,
    RADIO_OBSTACLES,
    TRAILS,
    WORLD_EXTENT,
    terrain_height_world,
)


TERRAIN_MODEL_DIR = PACKAGE_DIR / "models" / "hiking_lora_terrain"
TERRAIN_MESH_DIR = TERRAIN_MODEL_DIR / "meshes"
WORLD_PATH = PACKAGE_DIR / "worlds" / "hiking_mountain.sdf"
MESH_PATH = TERRAIN_MESH_DIR / "mountain_terrain.dae"


def fmt(value: float) -> str:
    return f"{value:.4f}".rstrip("0").rstrip(".")


def terrain_dae() -> str:
    grid = 73
    extent = WORLD_EXTENT
    step = (extent * 2.0) / (grid - 1)
    vertices = []
    for row in range(grid):
        y = -extent + row * step
        for col in range(grid):
            x = -extent + col * step
            vertices.append((x, y, terrain_height_world(x, y)))

    indices = []
    for row in range(grid - 1):
        for col in range(grid - 1):
            a = row * grid + col
            b = a + 1
            c = a + grid
            d = c + 1
            indices.extend((a, c, b, b, c, d))

    normals = [[0.0, 0.0, 0.0] for _ in vertices]
    for triangle in range(0, len(indices), 3):
        ia, ib, ic = indices[triangle : triangle + 3]
        ax, ay, az = vertices[ia]
        bx, by, bz = vertices[ib]
        cx, cy, cz = vertices[ic]
        ux, uy, uz = bx - ax, by - ay, bz - az
        vx, vy, vz = cx - ax, cy - ay, cz - az
        nx = uy * vz - uz * vy
        ny = uz * vx - ux * vz
        nz = ux * vy - uy * vx
        for index in (ia, ib, ic):
            normals[index][0] += nx
            normals[index][1] += ny
            normals[index][2] += nz

    normalized_normals = []
    for nx, ny, nz in normals:
        length = math.sqrt(nx * nx + ny * ny + nz * nz)
        if length <= 1e-9:
            normalized_normals.append((0.0, 0.0, 1.0))
        else:
            normalized_normals.append((nx / length, ny / length, nz / length))

    positions = " ".join(f"{fmt(x)} {fmt(y)} {fmt(z)}" for x, y, z in vertices)
    normal_values = " ".join(f"{fmt(x)} {fmt(y)} {fmt(z)}" for x, y, z in normalized_normals)
    faces = " ".join(f"{index} {index}" for index in indices)
    triangle_count = len(indices) // 3
    position_count = len(vertices) * 3
    normal_count = len(normalized_normals) * 3

    return f"""<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <contributor><authoring_tool>hiking_lora_sim procedural generator</authoring_tool></contributor>
    <unit name="meter" meter="1"/>
    <up_axis>Z_UP</up_axis>
  </asset>
  <library_geometries>
    <geometry id="mountain_terrain" name="mountain_terrain">
      <mesh>
        <source id="mountain_terrain_positions">
          <float_array id="mountain_terrain_positions_array" count="{position_count}">{positions}</float_array>
          <technique_common>
            <accessor source="#mountain_terrain_positions_array" count="{len(vertices)}" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <source id="mountain_terrain_normals">
          <float_array id="mountain_terrain_normals_array" count="{normal_count}">{normal_values}</float_array>
          <technique_common>
            <accessor source="#mountain_terrain_normals_array" count="{len(normalized_normals)}" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <vertices id="mountain_terrain_vertices">
          <input semantic="POSITION" source="#mountain_terrain_positions"/>
        </vertices>
        <triangles count="{triangle_count}">
          <input semantic="VERTEX" source="#mountain_terrain_vertices" offset="0"/>
          <input semantic="NORMAL" source="#mountain_terrain_normals" offset="1"/>
          <p>{faces}</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="Scene" name="Scene">
      <node id="mountain_terrain_node" name="mountain_terrain">
        <instance_geometry url="#mountain_terrain"/>
      </node>
    </visual_scene>
  </library_visual_scenes>
  <scene>
    <instance_visual_scene url="#Scene"/>
  </scene>
</COLLADA>
"""


def model_config() -> str:
    return """<?xml version="1.0"?>
<model>
  <name>hiking_lora_terrain</name>
  <version>1.0</version>
  <sdf version="1.10">model.sdf</sdf>
  <author>
    <name>hiking_lora_sim</name>
  </author>
  <description>Procedural wide mountain terrain mesh for the hiking LoRa simulation.</description>
</model>
"""


def model_sdf() -> str:
    return """<?xml version="1.0"?>
<sdf version="1.10">
  <model name="hiking_lora_terrain">
    <static>true</static>
    <link name="terrain_link">
      <collision name="terrain_collision">
        <geometry>
          <mesh>
            <uri>model://hiking_lora_terrain/meshes/mountain_terrain.dae</uri>
          </mesh>
        </geometry>
      </collision>
      <visual name="terrain_visual">
        <geometry>
          <mesh>
            <uri>model://hiking_lora_terrain/meshes/mountain_terrain.dae</uri>
          </mesh>
        </geometry>
        <material>
          <ambient>0.23 0.35 0.21 1</ambient>
          <diffuse>0.34 0.48 0.26 1</diffuse>
          <specular>0.08 0.08 0.07 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def box_visual(name: str, pose: str, size: str, color: str) -> str:
    ambient = " ".join(str(max(float(part) * 0.72, 0.0)) for part in color.split()[:3])
    return (
        f'        <visual name="{name}"><pose>{pose}</pose>'
        f"<geometry><box><size>{size}</size></box></geometry>"
        f"<material><ambient>{ambient} 1</ambient><diffuse>{color} 1</diffuse></material>"
        "</visual>\n"
    )


def cylinder_visual(name: str, pose: str, radius: float, length: float, color: str) -> str:
    ambient = " ".join(str(max(float(part) * 0.72, 0.0)) for part in color.split()[:3])
    return (
        f'        <visual name="{name}"><pose>{pose}</pose>'
        f"<geometry><cylinder><radius>{fmt(radius)}</radius><length>{fmt(length)}</length></cylinder></geometry>"
        f"<material><ambient>{ambient} 1</ambient><diffuse>{color} 1</diffuse></material>"
        "</visual>\n"
    )


def sphere_visual(name: str, pose: str, radius: float, color: str, alpha: float = 1.0) -> str:
    ambient = " ".join(str(max(float(part) * 0.72, 0.0)) for part in color.split()[:3])
    return (
        f'        <visual name="{name}"><pose>{pose}</pose>'
        f"<geometry><sphere><radius>{fmt(radius)}</radius></sphere></geometry>"
        f"<material><ambient>{ambient} {fmt(alpha)}</ambient><diffuse>{color} {fmt(alpha)}</diffuse></material>"
        "</visual>\n"
    )


def route_segments() -> str:
    colors = {
        "ridge_route": "0.72 0.47 0.20",
        "valley_route": "0.62 0.55 0.32",
        "crater_route": "0.63 0.34 0.22",
    }
    widths = {"ridge_route": 1.15, "valley_route": 0.92, "crater_route": 0.88}
    blocks = []
    for route_name, points in TRAILS.items():
        blocks.append(f'    <model name="trail_{route_name}">\n      <static>true</static>\n      <link name="trail_link">\n')
        visual_id = 0
        for start, end in zip(points, points[1:]):
            sx, sy = start
            ex, ey = end
            dx = ex - sx
            dy = ey - sy
            segment_length = math.hypot(dx, dy)
            chunks = max(1, int(segment_length / 5.0))
            yaw = math.atan2(dy, dx)
            for chunk in range(chunks):
                t0 = chunk / chunks
                t1 = (chunk + 1) / chunks
                mx = sx + dx * (t0 + t1) * 0.5
                my = sy + dy * (t0 + t1) * 0.5
                length = segment_length / chunks
                z = terrain_height_world(mx, my) + 0.14
                blocks.append(
                    box_visual(
                        f"{route_name}_{visual_id}",
                        f"{fmt(mx)} {fmt(my)} {fmt(z)} 0 0 {fmt(yaw)}",
                        f"{fmt(length)} {fmt(widths[route_name])} 0.08",
                        colors[route_name],
                    )
                )
                visual_id += 1
        blocks.append("      </link>\n    </model>\n")
    return "".join(blocks)


def forest_models() -> str:
    rng = random.Random(12)
    clusters = [
        ("lower_dense_forest", -55.0, -46.0, 27.0, 48),
        ("valley_forest", -38.0, 5.0, 25.0, 42),
        ("north_forest", 18.0, 37.0, 22.0, 30),
    ]
    blocks = []
    for cluster_name, cx, cy, radius, count in clusters:
        blocks.append(f'    <model name="{cluster_name}_trees">\n      <static>true</static>\n      <link name="trees">\n')
        for index in range(count):
            angle = rng.uniform(0.0, math.tau)
            distance = radius * math.sqrt(rng.uniform(0.0, 1.0))
            x = cx + math.cos(angle) * distance
            y = cy + math.sin(angle) * distance
            terrain_z = terrain_height_world(x, y)
            trunk = rng.uniform(1.7, 2.8)
            canopy = rng.uniform(0.85, 1.45)
            blocks.append(
                cylinder_visual(
                    f"trunk_{index}",
                    f"{fmt(x)} {fmt(y)} {fmt(terrain_z + trunk * 0.5)} 0 0 0",
                    rng.uniform(0.11, 0.19),
                    trunk,
                    "0.28 0.17 0.08",
                )
            )
            blocks.append(
                sphere_visual(
                    f"canopy_{index}",
                    f"{fmt(x)} {fmt(y)} {fmt(terrain_z + trunk + canopy * 0.55)} 0 0 0",
                    canopy,
                    "0.08 0.34 0.11",
                )
            )
        blocks.append("      </link>\n    </model>\n")
    return "".join(blocks)


def rock_models() -> str:
    rng = random.Random(21)
    clusters = [
        ("lower_cliff", 18.0, 16.0, 18.0, 20),
        ("crater_rocks", 55.0, 45.0, 18.0, 18),
        ("summit_boulders", 78.0, 72.0, 17.0, 16),
        ("western_scree", -68.0, -18.0, 19.0, 15),
    ]
    blocks = []
    for cluster_name, cx, cy, radius, count in clusters:
        blocks.append(f'    <model name="{cluster_name}">\n      <static>true</static>\n      <link name="rocks">\n')
        for index in range(count):
            angle = rng.uniform(0.0, math.tau)
            distance = radius * math.sqrt(rng.uniform(0.0, 1.0))
            x = cx + math.cos(angle) * distance
            y = cy + math.sin(angle) * distance
            sx = rng.uniform(1.4, 4.5)
            sy = rng.uniform(0.8, 2.8)
            sz = rng.uniform(0.8, 3.2)
            z = terrain_height_world(x, y) + sz * 0.48
            roll = rng.uniform(-0.22, 0.22)
            pitch = rng.uniform(-0.18, 0.18)
            yaw = rng.uniform(-math.pi, math.pi)
            blocks.append(
                box_visual(
                    f"rock_{index}",
                    f"{fmt(x)} {fmt(y)} {fmt(z)} {fmt(roll)} {fmt(pitch)} {fmt(yaw)}",
                    f"{fmt(sx)} {fmt(sy)} {fmt(sz)}",
                    "0.39 0.38 0.35",
                )
            )
        blocks.append("      </link>\n    </model>\n")
    return "".join(blocks)


def station_models() -> str:
    blocks = []
    for station in [BASE_STATION] + LORA_NODES:
        terrain_z = terrain_height_world(station.x, station.y)
        mast_height = 8.5 if station.kind == "base" else 6.0
        mast_radius = 0.17 if station.kind == "base" else 0.12
        mast_color = "0.0 0.18 0.9" if station.kind == "base" else "0.0 0.92 0.36"
        antenna_color = "0.0 0.35 1.0" if station.kind == "base" else "0.0 1.0 0.48"
        blocks.append(f'    <model name="{station.name}">\n      <static>true</static>\n      <pose>{fmt(station.x)} {fmt(station.y)} 0 0 0 0</pose>\n      <link name="tower">\n')
        blocks.append(
            cylinder_visual(
                "mast",
                f"0 0 {fmt(terrain_z + mast_height * 0.5)} 0 0 0",
                mast_radius,
                mast_height,
                mast_color,
            )
        )
        blocks.append(
            sphere_visual(
                "antenna",
                f"0 0 {fmt(terrain_z + mast_height + 0.35)} 0 0 0",
                0.42 if station.kind == "base" else 0.31,
                antenna_color,
            )
        )
        if station.kind == "base":
            blocks.append(
                box_visual(
                    "radio_panel",
                    f"0.75 -0.2 {fmt(terrain_z + 4.0)} 0 0.25 0",
                    "0.12 1.4 1.0",
                    "0.03 0.06 0.16",
                )
            )
        blocks.append("      </link>\n    </model>\n")
    return "".join(blocks)


def basecamp_model() -> str:
    x, y = BASE_STATION.x - 3.0, BASE_STATION.y + 2.0
    z = terrain_height_world(x, y)
    return f"""    <model name="base_camp">
      <static>true</static>
      <pose>{fmt(x)} {fmt(y)} 0 0 0 0</pose>
      <link name="camp_link">
{box_visual("platform", f"0 0 {fmt(z + 0.08)} 0 0 0", "13 9 0.16", "0.28 0.26 0.22")}{box_visual("hut", f"1.5 0.5 {fmt(z + 1.0)} 0 0 0", "3.2 2.6 2.0", "0.54 0.32 0.17")}{box_visual("roof", f"1.5 0.5 {fmt(z + 2.2)} 0 0 0.785", "3.5 3.0 0.45", "0.46 0.10 0.07")}{box_visual("equipment", f"-3.4 -1.8 {fmt(z + 0.55)} 0 0 0", "2.2 1.5 1.1", "0.24 0.32 0.42")}      </link>
    </model>
"""


def crater_model() -> str:
    cx, cy = 54.0, 45.0
    z = terrain_height_world(cx, cy)
    return f"""    <model name="crater_smoke_marker">
      <static>true</static>
      <pose>{fmt(cx)} {fmt(cy)} 0 0 0 0</pose>
      <link name="crater_marker">
{sphere_visual("bowl_shadow", f"0 0 {fmt(z + 0.6)} 0 0 0", 5.2, "0.11 0.10 0.09", 0.45)}{sphere_visual("steam_0", f"-2.0 1.0 {fmt(z + 4.6)} 0 0 0", 1.2, "0.72 0.72 0.67", 0.35)}{sphere_visual("steam_1", f"1.4 -1.0 {fmt(z + 5.8)} 0 0 0", 1.0, "0.82 0.82 0.76", 0.3)}      </link>
    </model>
"""


def hiker_model(active_route: str) -> str:
    start = TRAILS[active_route][0]
    x, y = start
    z = terrain_height_world(x, y) + 1.05
    waypoints = "\n".join(
        f"          <waypoint>{fmt(px)} {fmt(py)}</waypoint>" for px, py in TRAILS[active_route]
    )
    return f"""    <model name="hiker">
      <pose>{fmt(x)} {fmt(y)} {fmt(z)} 0 0 0.9</pose>
      <link name="body">
        <inertial>
          <mass>82</mass>
          <inertia><ixx>6.0</ixx><iyy>6.0</iyy><izz>2.0</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="body_collision">
          <pose>0 0 0 0 0 0</pose>
          <geometry><sphere><radius>0.42</radius></sphere></geometry>
        </collision>
        <visual name="legs"><pose>0 0 -0.45 0 0 0</pose><geometry><cylinder><radius>0.22</radius><length>0.75</length></cylinder></geometry><material><ambient>0.06 0.08 0.12 1</ambient><diffuse>0.08 0.10 0.16 1</diffuse></material></visual>
        <visual name="torso"><pose>0 0 0.18 0 0 0</pose><geometry><cylinder><radius>0.34</radius><length>0.92</length></cylinder></geometry><material><ambient>0.78 0.38 0.02 1</ambient><diffuse>1.0 0.54 0.06 1</diffuse></material></visual>
        <visual name="head"><pose>0 0 0.86 0 0 0</pose><geometry><sphere><radius>0.24</radius></sphere></geometry><material><ambient>0.55 0.38 0.25 1</ambient><diffuse>0.76 0.55 0.38 1</diffuse></material></visual>
        <visual name="backpack"><pose>-0.34 0 0.18 0 0 0</pose><geometry><box><size>0.24 0.55 0.75</size></box></geometry><material><ambient>0.05 0.15 0.10 1</ambient><diffuse>0.05 0.30 0.18 1</diffuse></material></visual>
      </link>
      <plugin filename="gz-sim-trajectory-follower-system" name="gz::sim::systems::TrajectoryFollower">
        <link_name>body</link_name>
        <loop>false</loop>
        <force>260</force>
        <torque>120</torque>
        <waypoints>
{waypoints}
        </waypoints>
      </plugin>
    </model>
"""


def terrain_model_instance() -> str:
    return """    <model name="wide_mountain_terrain">
      <static>true</static>
      <link name="terrain_link">
        <collision name="terrain_collision">
          <geometry>
            <mesh>
              <uri>model://hiking_lora_terrain/meshes/mountain_terrain.dae</uri>
            </mesh>
          </geometry>
        </collision>
        <visual name="terrain_visual">
          <geometry>
            <mesh>
              <uri>model://hiking_lora_terrain/meshes/mountain_terrain.dae</uri>
            </mesh>
          </geometry>
          <material>
            <ambient>0.23 0.35 0.21 1</ambient>
            <diffuse>0.34 0.48 0.26 1</diffuse>
            <specular>0.08 0.08 0.07 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""


def world_sdf(active_route: str) -> str:
    return f"""<?xml version="1.0"?>
<sdf version="1.10">
  <world name="hiking_lora_world">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <gravity>0 0 -9.81</gravity>
    <magnetic_field>5.5645e-06 2.28758e-05 -4.23884e-05</magnetic_field>
    <atmosphere type="adiabatic"/>

    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>-74 -168 86 0 0.52 0.43</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

    <scene>
      <ambient>0.45 0.48 0.48 1</ambient>
      <background>0.55 0.70 0.86 1</background>
      <grid>false</grid>
    </scene>

    <light name="sun" type="directional">
      <pose>0 0 120 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1.1</intensity>
      <direction>-0.42 0.32 -0.85</direction>
      <diffuse>0.92 0.88 0.76 1</diffuse>
      <specular>0.18 0.18 0.16 1</specular>
    </light>

{terrain_model_instance()}

{route_segments()}
{basecamp_model()}
{station_models()}
{forest_models()}
{rock_models()}
{crater_model()}
{hiker_model(active_route)}
  </world>
</sdf>
"""


def main() -> None:
    TERRAIN_MESH_DIR.mkdir(parents=True, exist_ok=True)
    WORLD_PATH.parent.mkdir(parents=True, exist_ok=True)
    MESH_PATH.write_text(terrain_dae(), encoding="utf-8")
    (TERRAIN_MODEL_DIR / "model.config").write_text(model_config(), encoding="utf-8")
    (TERRAIN_MODEL_DIR / "model.sdf").write_text(model_sdf(), encoding="utf-8")
    for route_name in TRAILS:
        route_world = PACKAGE_DIR / "worlds" / f"hiking_mountain_{route_name}.sdf"
        route_world.write_text(world_sdf(route_name), encoding="utf-8")
    WORLD_PATH.write_text(world_sdf("ridge_route"), encoding="utf-8")


if __name__ == "__main__":
    main()
