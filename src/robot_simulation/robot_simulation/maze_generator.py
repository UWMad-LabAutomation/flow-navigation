#!/usr/bin/env python3
"""Generate a random maze world and a matching Nav2 occupancy map."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import random
from typing import Optional


def generate_maze(width: int, height: int, seed: Optional[int] = None):
    """Generate a perfect maze using recursive backtracking."""
    if width < 5 or height < 5 or width % 2 == 0 or height % 2 == 0:
        raise ValueError('width and height must be odd integers greater than 4')

    actual_seed = seed if seed is not None else random.SystemRandom().randrange(
        0, 2**32)
    rng = random.Random(actual_seed)
    maze = [[1 for _ in range(width)] for _ in range(height)]
    directions = [(-2, 0), (2, 0), (0, -2), (0, 2)]

    def carve_passages(row: int, col: int):
        maze[row][col] = 0
        shuffled = directions[:]
        rng.shuffle(shuffled)
        for delta_row, delta_col in shuffled:
            new_row = row + delta_row
            new_col = col + delta_col
            if not (1 <= new_row < height - 1 and
                    1 <= new_col < width - 1):
                continue
            if maze[new_row][new_col] == 1:
                maze[row + delta_row // 2][col + delta_col // 2] = 0
                carve_passages(new_row, new_col)

    carve_passages(1, 1)
    return maze, actual_seed


def _cell_center(row: int, col: int, width: int, height: int,
                 cell_size: float):
    """Return the world x/y coordinate for a maze cell center."""
    x = (col - width / 2.0 + 0.5) * cell_size
    y = (height / 2.0 - row - 0.5) * cell_size
    return x, y


def _wall_model(maze, cell_size: float, wall_height: float):
    """Create one static SDF model containing every occupied maze cell."""
    height = len(maze)
    width = len(maze[0])
    parts = [
        '    <model name="maze_walls">',
        '      <static>true</static>',
        '      <link name="walls">',
    ]
    for row in range(height):
        for col in range(width):
            if maze[row][col] != 1:
                continue
            x, y = _cell_center(row, col, width, height, cell_size)
            pose = f'{x:.6f} {y:.6f} {wall_height / 2.0:.6f} 0 0 0'
            name = f'wall_{row}_{col}'
            geometry = (
                f'          <box><size>{cell_size:.6f} {cell_size:.6f} '
                f'{wall_height:.6f}</size></box>')
            parts.extend([
                f'        <collision name="{name}_collision">',
                f'          <pose>{pose}</pose>',
                '          <geometry>',
                geometry,
                '          </geometry>',
                '        </collision>',
                f'        <visual name="{name}_visual">',
                f'          <pose>{pose}</pose>',
                '          <geometry>',
                geometry,
                '          </geometry>',
                '          <material>',
                '            <ambient>0.35 0.35 0.38 1</ambient>',
                '            <diffuse>0.45 0.45 0.48 1</diffuse>',
                '          </material>',
                '        </visual>',
            ])
    parts.extend([
        '      </link>',
        '    </model>',
    ])
    return '\n'.join(parts)


def _write_map(maze, map_yaml: Path, map_image: Path, cell_size: float,
               resolution: float, seed: int):
    """Write a trinary PGM map with the same occupied cells as the world."""
    height = len(maze)
    width = len(maze[0])
    pixels_per_cell = round(cell_size / resolution)
    if pixels_per_cell < 1 or abs(pixels_per_cell * resolution - cell_size) > 1e-9:
        raise ValueError('cell_size must be an integer multiple of resolution')

    image_width = width * pixels_per_cell
    image_height = height * pixels_per_cell
    rows = []
    for maze_row in maze:
        pixel_row = bytearray()
        for cell in maze_row:
            value = 0 if cell == 1 else 255
            pixel_row.extend([value] * pixels_per_cell)
        for _ in range(pixels_per_cell):
            rows.append(bytes(pixel_row))

    map_image.write_bytes(
        f'P5\n{image_width} {image_height}\n255\n'.encode() +
        b''.join(rows))

    minimum = -width * cell_size / 2.0
    map_yaml.write_text(
        f'# maze_seed: {seed}\n'
        f'image: {map_image.name}\n'
        f'mode: trinary\n'
        f'resolution: {resolution:.6f}\n'
        f'origin: [{minimum:.6f}, {minimum:.6f}, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.196\n',
        encoding='utf-8')


def generate_artifacts(template_world: Path, output_dir: Path, width: int = 21,
                       height: int = 21, cell_size: float = 2.5,
                       resolution: float = 0.05, wall_height: float = 0.5,
                       seed: Optional[int] = None):
    """Generate SDF, PGM, and YAML files and return their paths and metadata."""
    maze, actual_seed = generate_maze(width, height, seed)
    output_dir.mkdir(parents=True, exist_ok=True)
    world_path = output_dir / 'maze_world.sdf'
    map_yaml = output_dir / 'maze.yaml'
    map_image = output_dir / 'maze.pgm'
    manifest_path = output_dir / 'maze_manifest.json'

    start_x, start_y = _cell_center(1, 1, width, height, cell_size)
    goal_x, goal_y = _cell_center(
        height - 2, width - 2, width, height, cell_size)
    world = template_world.read_text(encoding='utf-8')
    old_pose = '      <pose>0 0 0.080 0 0 0</pose>'
    new_pose = f'      <pose>{start_x:.6f} {start_y:.6f} 0.080 0 0 0</pose>'
    if old_pose not in world:
        raise ValueError('the FLOW world does not contain the expected base pose')
    world = world.replace(old_pose, new_pose, 1)
    marker = '    <model name="flow_base"'
    world = world.replace(marker, _wall_model(maze, cell_size, wall_height) +
                          '\n\n' + marker, 1)
    world_marker = '<world name="flow_base_world">'
    world_name_seed = str(actual_seed).replace('-', 'm')
    world = world.replace(
        world_marker,
        f'<!-- maze_seed: {actual_seed} -->\n'
        f'  <world name="flow_base_world_seed_{world_name_seed}">',
        1,
    )
    world_path.write_text(world, encoding='utf-8')
    _write_map(maze, map_yaml, map_image, cell_size, resolution, actual_seed)

    manifest_path.write_text(json.dumps({
        'seed': actual_seed,
        'width': width,
        'height': height,
        'cell_size': cell_size,
        'resolution': resolution,
        'wall_height': wall_height,
        'world': str(world_path),
        'map': str(map_yaml),
        'image': str(map_image),
    }, indent=2) + '\n', encoding='utf-8')

    return {
        'world': world_path,
        'map': map_yaml,
        'image': map_image,
        'manifest': manifest_path,
        'seed': actual_seed,
        'start': (start_x, start_y),
        'goal': (goal_x, goal_y),
        'corridor_width': cell_size,
    }


def main():
    """Generate maze artifacts from command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=Path('/tmp/flow_maze'))
    parser.add_argument('--template-world', type=Path, required=True)
    parser.add_argument('--width', type=int, default=21)
    parser.add_argument('--height', type=int, default=21)
    parser.add_argument('--cell-size', type=float, default=2.5)
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--wall-height', type=float, default=0.5)
    parser.add_argument('--seed', type=int)
    args = parser.parse_args()
    artifacts = generate_artifacts(
        template_world=args.template_world,
        output_dir=args.output_dir,
        width=args.width,
        height=args.height,
        cell_size=args.cell_size,
        resolution=args.resolution,
        wall_height=args.wall_height,
        seed=args.seed,
    )
    print(f"seed: {artifacts['seed']}")
    print(f"world: {artifacts['world']}")
    print(f"map: {artifacts['map']}")
    print(f"start: {artifacts['start'][0]:.3f}, {artifacts['start'][1]:.3f}")
    print(f"goal: {artifacts['goal'][0]:.3f}, {artifacts['goal'][1]:.3f}")


if __name__ == '__main__':
    main()
