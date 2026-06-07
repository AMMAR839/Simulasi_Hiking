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
TRAIL_SURFACE_MESH_PATH = TERRAIN_MESH_DIR / "trail_surface.dae"
STREAM_SURFACE_MESH_PATH = TERRAIN_MESH_DIR / "stream_surface.dae"

# Color palette
_ROCK_DARK   = "0.30 0.28 0.26"
_ROCK_MID    = "0.44 0.41 0.37"
_ROCK_LIGHT  = "0.56 0.53 0.48"
_TRAIL_SURFACE_WIDTHS = {
    "ridge_route": 2.2,
    "valley_route": 2.0,
    "crater_route": 2.0,
}

_STREAM_SURFACE_WIDTH = 1.55
_STREAM_POINTS = [
    (-101.0, -92.0),
    (-91.0, -73.0),
    (-80.0, -53.0),
    (-66.0, -32.0),
    (-52.0, -10.0),
    (-34.0, 6.0),
    (-12.0, 20.0),
    (10.0, 30.0),
    (31.0, 39.0),
]


def fmt(value: float) -> str:
    return f"{value:.4f}".rstrip("0").rstrip(".")


def terrain_dae() -> str:
    grid = 113
    extent = WORLD_EXTENT
    step = (extent * 2.0) / (grid - 1)
    vertices = []
    for row in range(grid):
        y = -extent + row * step
        for col in range(grid):
            x = -extent + col * step
            vertices.append((x, y, terrain_height_world(x, y)))

    triangles = []
    for row in range(grid - 1):
        for col in range(grid - 1):
            a = row * grid + col
            b = a + 1
            c = a + grid
            d = c + 1
            triangles.append((a, b, c))
            triangles.append((b, d, c))

    normals = [[0.0, 0.0, 0.0] for _ in vertices]
    for ia, ib, ic in triangles:
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
    faces = " ".join(f"{index} {index}" for triangle in triangles for index in triangle)
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
        <triangles count="{len(triangles)}">
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
          <ambient>0.22 0.32 0.17 1</ambient>
          <diffuse>0.33 0.46 0.24 1</diffuse>
          <specular>0.06 0.07 0.05 1</specular>
        </material>
      </visual>
      <visual name="trail_surface_visual">
        <geometry>
          <mesh>
            <uri>model://hiking_lora_terrain/meshes/trail_surface.dae</uri>
          </mesh>
        </geometry>
        <material>
          <ambient>0.31 0.22 0.13 1</ambient>
          <diffuse>0.48 0.36 0.22 1</diffuse>
          <specular>0.04 0.03 0.02 1</specular>
        </material>
      </visual>
      <visual name="stream_surface_visual">
        <geometry>
          <mesh>
            <uri>model://hiking_lora_terrain/meshes/stream_surface.dae</uri>
          </mesh>
        </geometry>
        <material>
          <ambient>0.05 0.18 0.32 0.72</ambient>
          <diffuse>0.10 0.34 0.58 0.72</diffuse>
          <specular>0.02 0.04 0.06 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def box_visual(name: str, pose: str, size: str, color: str, alpha: float = 1.0) -> str:
    ambient = " ".join(str(max(float(part) * 0.70, 0.0)) for part in color.split()[:3])
    return (
        f'        <visual name="{name}"><pose>{pose}</pose>'
        f"<geometry><box><size>{size}</size></box></geometry>"
        f"<material><ambient>{ambient} {fmt(alpha)}</ambient><diffuse>{color} {fmt(alpha)}</diffuse></material>"
        "</visual>\n"
    )


def cylinder_visual(name: str, pose: str, radius: float, length: float, color: str, alpha: float = 1.0) -> str:
    ambient = " ".join(str(max(float(part) * 0.70, 0.0)) for part in color.split()[:3])
    return (
        f'        <visual name="{name}"><pose>{pose}</pose>'
        f"<geometry><cylinder><radius>{fmt(radius)}</radius><length>{fmt(length)}</length></cylinder></geometry>"
        f"<material><ambient>{ambient} {fmt(alpha)}</ambient><diffuse>{color} {fmt(alpha)}</diffuse></material>"
        "</visual>\n"
    )


def sphere_visual(name: str, pose: str, radius: float, color: str, alpha: float = 1.0) -> str:
    ambient = " ".join(str(max(float(part) * 0.70, 0.0)) for part in color.split()[:3])
    return (
        f'        <visual name="{name}"><pose>{pose}</pose>'
        f"<geometry><sphere><radius>{fmt(radius)}</radius></sphere></geometry>"
        f"<material><ambient>{ambient} {fmt(alpha)}</ambient><diffuse>{color} {fmt(alpha)}</diffuse></material>"
        "</visual>\n"
    )


def include_model(name: str, model_name: str, pose: str) -> str:
    return (
        "    <include>\n"
        f"      <uri>model://{model_name}</uri>\n"
        f"      <name>{name}</name>\n"
        f"      <pose>{pose}</pose>\n"
        "    </include>\n"
    )


def catmull_rom_point(p0, p1, p2, p3, t: float):
    t2 = t * t
    t3 = t2 * t
    x = (
        (2.0 * p1[0])
        + (-p0[0] + p2[0]) * t
        + (2.0 * p0[0] - 5.0 * p1[0] + 4.0 * p2[0] - p3[0]) * t2
        + (-p0[0] + 3.0 * p1[0] - 3.0 * p2[0] + p3[0]) * t3
    )
    y = (
        (2.0 * p1[1])
        + (-p0[1] + p2[1]) * t
        + (2.0 * p0[1] - 5.0 * p1[1] + 4.0 * p2[1] - p3[1]) * t2
        + (-p0[1] + 3.0 * p1[1] - 3.0 * p2[1] + p3[1]) * t3
    )
    return (
        0.5 * x,
        0.5 * y,
    )


def smooth_polyline(points, target_step: float):
    if len(points) < 3:
        return list(points)

    smoothed = []
    for index in range(len(points) - 1):
        p0 = points[max(index - 1, 0)]
        p1 = points[index]
        p2 = points[index + 1]
        p3 = points[min(index + 2, len(points) - 1)]
        segment_length = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        samples = max(4, math.ceil(segment_length / target_step))
        if index == 0:
            smoothed.append(p1)
        for sample in range(1, samples + 1):
            smoothed.append(catmull_rom_point(p0, p1, p2, p3, sample / samples))
    return smoothed


def surface_strip_dae(name: str, strips, z_offset: float) -> str:
    vertices = []
    triangles = []

    for points, width, target_step in strips:
        smooth_points = smooth_polyline(points, target_step=target_step)
        if len(smooth_points) < 2:
            continue

        strip_indices = []
        half_width = width * 0.5
        for index, (x, y) in enumerate(smooth_points):
            if index == 0:
                tx = smooth_points[1][0] - x
                ty = smooth_points[1][1] - y
            elif index == len(smooth_points) - 1:
                tx = x - smooth_points[index - 1][0]
                ty = y - smooth_points[index - 1][1]
            else:
                tx = smooth_points[index + 1][0] - smooth_points[index - 1][0]
                ty = smooth_points[index + 1][1] - smooth_points[index - 1][1]

            length = math.hypot(tx, ty)
            if length <= 1e-9:
                nx, ny = 0.0, 1.0
            else:
                nx, ny = -ty / length, tx / length

            lx = x + nx * half_width
            ly = y + ny * half_width
            rx = x - nx * half_width
            ry = y - ny * half_width
            left_index = len(vertices)
            vertices.append((lx, ly, terrain_height_world(lx, ly) + z_offset))
            vertices.append((rx, ry, terrain_height_world(rx, ry) + z_offset))
            strip_indices.append((left_index, left_index + 1))

        for (left_a, right_a), (left_b, right_b) in zip(strip_indices, strip_indices[1:]):
            triangles.append((left_a, right_a, left_b))
            triangles.append((right_a, right_b, left_b))

    positions = " ".join(f"{fmt(x)} {fmt(y)} {fmt(z)}" for x, y, z in vertices)
    normal_values = " ".join("0 0 1" for _ in vertices)
    faces = " ".join(f"{index} {index}" for triangle in triangles for index in triangle)
    position_count = len(vertices) * 3
    normal_count = len(vertices) * 3

    return f"""<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <contributor><authoring_tool>hiking_lora_sim procedural generator</authoring_tool></contributor>
    <unit name="meter" meter="1"/>
    <up_axis>Z_UP</up_axis>
  </asset>
  <library_geometries>
    <geometry id="{name}" name="{name}">
      <mesh>
        <source id="{name}_positions">
          <float_array id="{name}_positions_array" count="{position_count}">{positions}</float_array>
          <technique_common>
            <accessor source="#{name}_positions_array" count="{len(vertices)}" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <source id="{name}_normals">
          <float_array id="{name}_normals_array" count="{normal_count}">{normal_values}</float_array>
          <technique_common>
            <accessor source="#{name}_normals_array" count="{len(vertices)}" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <vertices id="{name}_vertices">
          <input semantic="POSITION" source="#{name}_positions"/>
        </vertices>
        <triangles count="{len(triangles)}">
          <input semantic="VERTEX" source="#{name}_vertices" offset="0"/>
          <input semantic="NORMAL" source="#{name}_normals" offset="1"/>
          <p>{faces}</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="Scene" name="Scene">
      <node id="{name}_node" name="{name}">
        <instance_geometry url="#{name}"/>
      </node>
    </visual_scene>
  </library_visual_scenes>
  <scene>
    <instance_visual_scene url="#Scene"/>
  </scene>
</COLLADA>
"""


def trail_surface_dae() -> str:
    strips = [
        (points, _TRAIL_SURFACE_WIDTHS[route_name], 0.65)
        for route_name, points in TRAILS.items()
    ]
    return surface_strip_dae("trail_surface", strips, z_offset=0.085)


def stream_surface_dae() -> str:
    return surface_strip_dae(
        "stream_surface",
        [(_STREAM_POINTS, _STREAM_SURFACE_WIDTH, 0.55)],
        z_offset=0.095,
    )


def route_segments() -> str:
    colors = {
        "ridge_route":  "0.58 0.39 0.20",
        "valley_route": "0.48 0.36 0.22",
        "crater_route": "0.52 0.30 0.20",
    }
    widths = {"ridge_route": 0.68, "valley_route": 0.62, "crater_route": 0.58}
    blocks = []
    for route_name, points in TRAILS.items():
        blocks.append(
            f'    <model name="trail_{route_name}">\n'
            f'      <static>true</static>\n'
            f'      <link name="trail_link">\n'
        )
        visual_id = 0
        smooth_points = smooth_polyline(points, target_step=1.15)
        for start, end in zip(smooth_points, smooth_points[1:]):
            sx, sy = start
            ex, ey = end
            dx = ex - sx
            dy = ey - sy
            length = math.hypot(dx, dy)
            if length <= 1e-6:
                continue
            yaw = math.atan2(dy, dx)
            mx = sx + dx * 0.5
            my = sy + dy * 0.5
            z = terrain_height_world(mx, my) + 0.06
            blocks.append(
                box_visual(
                    f"{route_name}_{visual_id}",
                    f"{fmt(mx)} {fmt(my)} {fmt(z)} 0 0 {fmt(yaw)}",
                    f"{fmt(length * 1.16)} {fmt(widths[route_name])} 0.03",
                    colors[route_name],
                )
            )
            visual_id += 1
        blocks.append("      </link>\n    </model>\n")
    return "".join(blocks)


def forest_models() -> str:
    rng = random.Random(12)
    clusters = [
        ("lower_dense_forest", -55.0, -46.0, 34.0, 58),
        ("valley_forest",      -38.0,   5.0, 31.0, 56),
        ("north_forest",        18.0,  37.0, 27.0, 38),
        ("mid_slope_east",      28.0,  14.0, 21.0, 28),
        ("south_approach",     -74.0, -62.0, 25.0, 34),
        ("ridge_treeline",       8.0,  -8.0, 20.0, 18),
        ("upper_saddle_forest",  55.0,  61.0, 16.0, 8),
    ]
    blocks = []
    for cluster_name, cx, cy, radius, count in clusters:
        for i in range(count):
            angle = rng.uniform(0.0, math.tau)
            dist = radius * math.sqrt(rng.uniform(0.0, 1.0))
            x = cx + math.cos(angle) * dist
            y = cy + math.sin(angle) * dist
            z = terrain_height_world(x, y)
            yaw = rng.uniform(-math.pi, math.pi)
            blocks.append(
                include_model(
                    f"{cluster_name}_pine_tree_{i}",
                    "hiking_lora_pine_tree",
                    f"{fmt(x)} {fmt(y)} {fmt(z)} 0 0 {fmt(yaw)}",
                )
            )
    return "".join(blocks)


def ground_cover_models() -> str:
    rng = random.Random(34)
    shrub_clusters = [
        ("basecamp_scrub", -92.0, -83.0, 28.0, 28),
        ("valley_scrub", -44.0, -4.0, 34.0, 40),
        ("ridge_scrub", 7.0, -8.0, 28.0, 34),
        ("crater_scrub", 48.0, 43.0, 24.0, 24),
        ("summit_scrub", 72.0, 65.0, 22.0, 20),
    ]
    blocks = [
        '    <model name="low_ground_cover">\n'
        '      <static>true</static>\n'
        '      <link name="cover">\n'
    ]
    visual_id = 0
    for _, cx, cy, radius, count in shrub_clusters:
        for _ in range(count):
            angle = rng.uniform(0.0, math.tau)
            dist = radius * math.sqrt(rng.uniform(0.0, 1.0))
            x = cx + math.cos(angle) * dist
            y = cy + math.sin(angle) * dist
            z = terrain_height_world(x, y)
            color = rng.choice(["0.06 0.26 0.08", "0.10 0.32 0.10", "0.12 0.28 0.06"])
            if rng.random() < 0.65:
                radius_v = rng.uniform(0.35, 0.80)
                blocks.append(
                    sphere_visual(
                        f"shrub_{visual_id}",
                        f"{fmt(x)} {fmt(y)} {fmt(z + radius_v * 0.42)} 0 0 0",
                        radius_v,
                        color,
                    )
                )
            else:
                h = rng.uniform(0.35, 0.85)
                blocks.append(
                    cylinder_visual(
                        f"grass_{visual_id}",
                        f"{fmt(x)} {fmt(y)} {fmt(z + h * 0.45)} 0 0 0",
                        rng.uniform(0.12, 0.24),
                        h,
                        color,
                    )
                )
            visual_id += 1

    for i in range(24):
        x = rng.uniform(-95.0, 55.0)
        y = rng.uniform(-80.0, 45.0)
        z = terrain_height_world(x, y)
        yaw = rng.uniform(-math.pi, math.pi)
        blocks.append(
            cylinder_visual(
                f"fallen_log_{i}",
                f"{fmt(x)} {fmt(y)} {fmt(z + 0.20)} 0 1.5708 {fmt(yaw)}",
                rng.uniform(0.10, 0.18),
                rng.uniform(1.8, 4.2),
                "0.20 0.11 0.05",
            )
        )

    blocks.append("      </link>\n    </model>\n")
    return "".join(blocks)


def rock_models() -> str:
    rng = random.Random(21)
    clusters = [
        ("lower_cliff",      18.0,  16.0, 20.0, 24),
        ("crater_rocks",     55.0,  45.0, 20.0, 22),
        ("summit_boulders",  78.0,  72.0, 18.0, 20),
        ("western_scree",   -68.0, -18.0, 21.0, 18),
        ("ridge_talus",      42.0,  28.0, 16.0, 16),
    ]
    _rock_colors = [_ROCK_DARK, _ROCK_MID, _ROCK_LIGHT, _ROCK_MID, _ROCK_DARK]
    blocks = []
    for cluster_name, cx, cy, radius, count in clusters:
        blocks.append(
            f'    <model name="{cluster_name}">\n'
            f'      <static>true</static>\n'
            f'      <link name="rocks">\n'
        )
        for i in range(count):
            angle = rng.uniform(0.0, math.tau)
            dist = radius * math.sqrt(rng.uniform(0.0, 1.0))
            x = cx + math.cos(angle) * dist
            y = cy + math.sin(angle) * dist
            sx = rng.uniform(1.5, 5.0)
            sy = rng.uniform(0.9, 3.2)
            sz = rng.uniform(0.9, 3.6)
            z = terrain_height_world(x, y)
            roll  = rng.uniform(-0.28, 0.28)
            pitch = rng.uniform(-0.22, 0.22)
            yaw   = rng.uniform(-math.pi, math.pi)
            color = _rock_colors[rng.randrange(len(_rock_colors))]
            # Large boulders → sphere; smaller rocks → box
            if sx > 3.5 and rng.random() < 0.40:
                r = (sx + sz) * 0.26
                blocks.append(sphere_visual(
                    f"rock_{i}",
                    f"{fmt(x)} {fmt(y)} {fmt(z + r * 0.72)} {fmt(roll)} {fmt(pitch)} {fmt(yaw)}",
                    r, color,
                ))
            else:
                blocks.append(box_visual(
                    f"rock_{i}",
                    f"{fmt(x)} {fmt(y)} {fmt(z + sz * 0.48)} {fmt(roll)} {fmt(pitch)} {fmt(yaw)}",
                    f"{fmt(sx)} {fmt(sy)} {fmt(sz)}",
                    color,
                ))
        blocks.append("      </link>\n    </model>\n")
    return "".join(blocks)


def stream_model() -> str:
    blocks = [
        '    <model name="valley_stream">\n'
        '      <static>true</static>\n'
        '      <link name="water">\n'
    ]
    visual_id = 0
    smooth_points = smooth_polyline(_STREAM_POINTS, target_step=0.9)
    for start, end in zip(smooth_points, smooth_points[1:]):
        sx, sy = start
        ex, ey = end
        dx = ex - sx
        dy = ey - sy
        length = math.hypot(dx, dy)
        if length <= 1e-6:
            continue
        yaw = math.atan2(dy, dx)
        mx = sx + dx * 0.5
        my = sy + dy * 0.5
        z = terrain_height_world(mx, my) + 0.065
        blocks.append(
            box_visual(
                f"stream_{visual_id}",
                f"{fmt(mx)} {fmt(my)} {fmt(z)} 0 0 {fmt(yaw)}",
                f"{fmt(length * 1.20)} 1.45 0.032",
                "0.10 0.34 0.58",
                0.58,
            )
        )
        visual_id += 1
    blocks.append("      </link>\n    </model>\n")
    return "".join(blocks)


def trail_marker_models() -> str:
    route_colors = {
        "ridge_route": "0.95 0.58 0.12",
        "valley_route": "0.30 0.70 0.22",
        "crater_route": "0.85 0.25 0.12",
    }
    blocks = [
        '    <model name="trail_signs_and_markers">\n'
        '      <static>true</static>\n'
        '      <link name="signs">\n'
    ]
    marker_id = 0
    for route_name, points in TRAILS.items():
        color = route_colors[route_name]
        selected = [0, max(1, len(points) // 3), max(2, 2 * len(points) // 3), len(points) - 1]
        for index in selected:
            x, y = points[index]
            z = terrain_height_world(x, y)
            offset = 1.65 if index % 2 == 0 else -1.65
            blocks.append(
                cylinder_visual(
                    f"sign_post_{marker_id}",
                    f"{fmt(x + offset)} {fmt(y + 0.7)} {fmt(z + 0.65)} 0 0 0",
                    0.055,
                    1.30,
                    "0.27 0.16 0.07",
                )
            )
            blocks.append(
                box_visual(
                    f"sign_board_{marker_id}",
                    f"{fmt(x + offset)} {fmt(y + 0.7)} {fmt(z + 1.28)} 0 0 0.18",
                    "0.10 0.95 0.36",
                    color,
                )
            )
            blocks.append(
                sphere_visual(
                    f"marker_dot_{marker_id}",
                    f"{fmt(x)} {fmt(y)} {fmt(z + 0.36)} 0 0 0",
                    0.18,
                    color,
                )
            )
            marker_id += 1
    blocks.append("      </link>\n    </model>\n")
    return "".join(blocks)


def station_models() -> str:
    blocks = []
    for station in [BASE_STATION] + LORA_NODES:
        terrain_z = terrain_height_world(station.x, station.y)
        is_base = station.kind == "base"
        mast_height = 6.0 if is_base else 3.6
        mast_radius = 0.12 if is_base else 0.075
        mast_color = "0.05 0.18 0.55" if is_base else "0.12 0.42 0.18"
        ant_color = "0.12 0.42 0.95" if is_base else "0.12 0.85 0.34"
        plat_size = 2.4 if is_base else 1.15

        blocks.append(
            f'    <model name="{station.name}">\n'
            f'      <static>true</static>\n'
            f'      <pose>{fmt(station.x)} {fmt(station.y)} 0 0 0 0</pose>\n'
            f'      <link name="tower">\n'
        )

        # Ground platform / foundation
        blocks.append(box_visual(
            "foundation",
            f"0 0 {fmt(terrain_z + 0.08)} 0 0 0",
            f"{fmt(plat_size)} {fmt(plat_size)} 0.16",
            "0.16 0.16 0.15",
        ))

        # Vertical mast
        blocks.append(cylinder_visual(
            "mast",
            f"0 0 {fmt(terrain_z + mast_height * 0.5 + 0.16)} 0 0 0",
            mast_radius, mast_height, mast_color,
        ))

        # Small antenna marker on top
        ant_z = terrain_z + mast_height + 0.28
        blocks.append(sphere_visual(
            "antenna",
            f"0 0 {fmt(ant_z)} 0 0 0",
            0.28 if is_base else 0.18, ant_color,
        ))

        if is_base:
            blocks.append(box_visual(
                "shelter",
                f"0.92 0.70 {fmt(terrain_z + 0.46)} 0 0 0",
                "0.70 0.55 0.76", "0.15 0.16 0.18",
            ))
        else:
            blocks.append(box_visual(
                "relay_box",
                f"0.24 0.0 {fmt(terrain_z + 1.05)} 0 0 0",
                "0.32 0.24 0.42", "0.12 0.13 0.12",
            ))

        blocks.append("      </link>\n    </model>\n")
    return "".join(blocks)


def basecamp_model() -> str:
    x, y = BASE_STATION.x - 3.5, BASE_STATION.y + 2.5
    z = terrain_height_world(x, y)
    return (
        f'    <model name="base_camp">\n'
        f'      <static>true</static>\n'
        f'      <pose>{fmt(x)} {fmt(y)} 0 0 0 0</pose>\n'
        f'      <link name="camp_link">\n'
        + box_visual("platform", f"0 0 {fmt(z + 0.09)} 0 0 0", "14.0 10.0 0.18", "0.26 0.24 0.20")
        + box_visual("hut_main", f"1.8 0.6 {fmt(z + 1.1)} 0 0 0", "3.4 2.8 2.2", "0.52 0.30 0.16")
        + box_visual("roof",     f"1.8 0.6 {fmt(z + 2.42)} 0 0 0.785", "3.7 3.2 0.48", "0.44 0.09 0.06")
        + box_visual("hut_small", f"-2.2 -1.5 {fmt(z + 0.75)} 0 0 0.2", "2.2 1.6 1.5", "0.46 0.28 0.14")
        + box_visual("roof_small", f"-2.2 -1.5 {fmt(z + 1.58)} 0 0 0.2", "2.4 1.8 0.35", "0.44 0.09 0.06")
        + box_visual("equipment_a", f"-4.2 -2.0 {fmt(z + 0.60)} 0 0 0", "2.4 1.6 1.2", "0.22 0.30 0.40")
        + box_visual("equipment_b", f"-4.2 1.5 {fmt(z + 0.45)} 0 0 0", "1.8 1.4 0.9", "0.24 0.28 0.38")
        + box_visual("bench", f"3.8 0.6 {fmt(z + 0.46)} 0 0 0", "1.0 2.4 0.5", "0.35 0.22 0.10")
        + "      </link>\n    </model>\n"
    )


def crater_model() -> str:
    cx, cy = 54.0, 45.0
    z = terrain_height_world(cx, cy)
    return (
        f'    <model name="crater_smoke_marker">\n'
        f'      <static>true</static>\n'
        f'      <pose>{fmt(cx)} {fmt(cy)} 0 0 0 0</pose>\n'
        f'      <link name="crater_marker">\n'
        + sphere_visual("bowl_shadow", f"0 0 {fmt(z + 0.7)} 0 0 0", 5.5, "0.10 0.09 0.08", 0.50)
        + sphere_visual("rim_rock_0",  f"8.0 4.0 {fmt(z + 2.8)} 0 0 0", 2.2, _ROCK_DARK)
        + sphere_visual("rim_rock_1",  f"-6.0 7.0 {fmt(z + 2.4)} 0 0 0", 1.8, _ROCK_MID)
        + sphere_visual("rim_rock_2",  f"4.0 -8.0 {fmt(z + 2.6)} 0 0 0", 2.0, _ROCK_DARK)
        + sphere_visual("steam_0",  f"-2.0  1.5 {fmt(z + 5.2)} 0 0 0", 1.4, "0.80 0.80 0.76", 0.32)
        + sphere_visual("steam_1",  f" 1.8 -0.8 {fmt(z + 6.8)} 0 0 0", 1.1, "0.88 0.88 0.82", 0.26)
        + sphere_visual("steam_2",  f" 0.5  2.2 {fmt(z + 8.0)} 0 0 0", 0.8, "0.92 0.92 0.88", 0.20)
        + "      </link>\n    </model>\n"
    )


def snow_cap_model() -> str:
    # Sparse snow patches at high summit area (74, 70)
    sx, sy = 74.0, 70.0
    sz = terrain_height_world(sx, sy)
    blocks = (
        f'    <model name="summit_snow">\n'
        f'      <static>true</static>\n'
        f'      <link name="snow">\n'
        + sphere_visual("snow_main",  f"{fmt(sx)} {fmt(sy)} {fmt(sz + 2.0)} 0 0 0", 7.0, "0.95 0.96 0.98", 0.80)
        + sphere_visual("snow_patch0", f"{fmt(sx - 5)} {fmt(sy + 3)} {fmt(terrain_height_world(sx-5, sy+3) + 1.2)} 0 0 0", 3.5, "0.92 0.93 0.96", 0.65)
        + sphere_visual("snow_patch1", f"{fmt(sx + 6)} {fmt(sy - 4)} {fmt(terrain_height_world(sx+6, sy-4) + 1.0)} 0 0 0", 3.0, "0.90 0.92 0.95", 0.60)
        + "      </link>\n    </model>\n"
    )
    return blocks


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
          <ambient>0.22 0.32 0.17 1</ambient>
          <diffuse>0.33 0.46 0.24 1</diffuse>
          <specular>0.06 0.07 0.05 1</specular>
        </material>
      </visual>
      <visual name="trail_surface_visual">
        <geometry>
          <mesh>
            <uri>model://hiking_lora_terrain/meshes/trail_surface.dae</uri>
          </mesh>
        </geometry>
        <material>
          <ambient>0.31 0.22 0.13 1</ambient>
          <diffuse>0.48 0.36 0.22 1</diffuse>
          <specular>0.04 0.03 0.02 1</specular>
        </material>
      </visual>
      <visual name="stream_surface_visual">
        <geometry>
          <mesh>
            <uri>model://hiking_lora_terrain/meshes/stream_surface.dae</uri>
          </mesh>
        </geometry>
        <material>
          <ambient>0.05 0.18 0.32 0.72</ambient>
          <diffuse>0.10 0.34 0.58 0.72</diffuse>
          <specular>0.02 0.04 0.06 1</specular>
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
        <pose>-132 -128 96 0 0.82 0.72</pose>
        <view_controller>orbit</view_controller>
      </camera>
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </gz-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.4 0.4 0.4</ambient_light>
        <background_color>0.8 0.8 0.8</background_color>
        <camera_pose>-132 -128 96 0 0.82 0.72</camera_pose>
      </plugin>
      <plugin filename="GzSceneManager" name="Scene Manager">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="InteractiveViewControl" name="Interactive view control">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="HikingLoraKeyPlugin" name="Hiking LoRa Keyboard">
        <gz-gui>
          <title>Hiking LoRa Keyboard</title>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">320</property>
          <property key="height" type="double">78</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">true</property>
        </gz-gui>
        <manual_command_duration_s>0.15</manual_command_duration_s>
        <overview_camera_xy_step>8.0</overview_camera_xy_step>
        <overview_camera_z_step>10.0</overview_camera_z_step>
        <overview_camera_angle_step_rad>0.10</overview_camera_angle_step_rad>
      </plugin>
      <plugin filename="CameraTracking" name="Camera Tracking">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="WorldControl" name="World control">
        <gz-gui>
          <title>World control</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">72</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="left" target="left"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </gz-gui>
        <play_pause>true</play_pause>
        <step>true</step>
        <start_paused>false</start_paused>
        <use_event>true</use_event>
      </plugin>
      <plugin filename="WorldStats" name="World stats">
        <gz-gui>
          <title>World stats</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">110</property>
          <property type="double" key="width">290</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="right" target="right"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </gz-gui>
        <sim_time>true</sim_time>
        <real_time>true</real_time>
        <real_time_factor>true</real_time_factor>
        <iterations>true</iterations>
      </plugin>
    </gui>

    <scene>
      <ambient>0.42 0.46 0.48 1</ambient>
      <background>0.50 0.68 0.88 1</background>
      <grid>false</grid>
      <fog>
        <type>linear</type>
        <color>0.58 0.67 0.74 1</color>
        <density>0.0007</density>
        <start>120</start>
        <end>360</end>
      </fog>
    </scene>

    <!-- Primary sun — warm morning light from south-east -->
    <light name="sun" type="directional">
      <pose>0 0 200 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1.05</intensity>
      <direction>-0.38 0.28 -0.88</direction>
      <diffuse>0.95 0.90 0.78 1</diffuse>
      <specular>0.20 0.18 0.15 1</specular>
    </light>

    <!-- Sky fill — cool blue fill from opposite side to soften shadows -->
    <light name="sky_fill" type="directional">
      <pose>0 0 150 0 0 0</pose>
      <cast_shadows>false</cast_shadows>
      <intensity>0.38</intensity>
      <direction>0.30 -0.40 -0.87</direction>
      <diffuse>0.58 0.68 0.85 1</diffuse>
      <specular>0.04 0.05 0.06 1</specular>
    </light>

{terrain_model_instance()}

{trail_marker_models()}
{basecamp_model()}
{station_models()}
{forest_models()}
{ground_cover_models()}
{rock_models()}
{crater_model()}
{snow_cap_model()}
  </world>
</sdf>
"""


def main() -> None:
    TERRAIN_MESH_DIR.mkdir(parents=True, exist_ok=True)
    WORLD_PATH.parent.mkdir(parents=True, exist_ok=True)
    MESH_PATH.write_text(terrain_dae(), encoding="utf-8")
    TRAIL_SURFACE_MESH_PATH.write_text(trail_surface_dae(), encoding="utf-8")
    STREAM_SURFACE_MESH_PATH.write_text(stream_surface_dae(), encoding="utf-8")
    (TERRAIN_MODEL_DIR / "model.config").write_text(model_config(), encoding="utf-8")
    (TERRAIN_MODEL_DIR / "model.sdf").write_text(model_sdf(), encoding="utf-8")
    for route_name in TRAILS:
        route_world = PACKAGE_DIR / "worlds" / f"hiking_mountain_{route_name}.sdf"
        route_world.write_text(world_sdf(route_name), encoding="utf-8")
    WORLD_PATH.write_text(world_sdf("ridge_route"), encoding="utf-8")


if __name__ == "__main__":
    main()
