# Copyright 2026 Zach Gaier
# Licensed under the Apache License, Version 2.0.

"""Check that fewer wall boxes preserve the exact occupied maze cells."""

import xml.etree.ElementTree as ET

import pytest
from robot_simulation.maze_generator import _cell_center, _wall_model, generate_maze


@pytest.mark.parametrize('seed', [42, 1400869175, 2274250215])
def test_merged_walls_preserve_collision_and_visual_coverage(seed):
    """Cover every wall once without covering any corridor or changing height."""
    maze, _ = generate_maze(21, 21, seed)
    model = ET.fromstring(_wall_model(maze, 2.5, 0.5))
    collisions = model.findall('link/collision')
    visuals = model.findall('link/visual')
    assert len(collisions) == len(visuals) < sum(map(sum, maze))
    for collision, visual in zip(collisions, visuals):
        assert collision.findtext('pose') == visual.findtext('pose')
        assert collision.findtext('geometry/box/size') == visual.findtext('geometry/box/size')
    boxes = []
    for collision in collisions:
        x, y, z, *_ = map(float, collision.findtext('pose').split())
        sx, sy, sz = map(float, collision.findtext('geometry/box/size').split())
        assert sy == 2.5 and sz == 0.5 and z == 0.25
        boxes.append((x, y, sx, sy))
    for row, cells in enumerate(maze):
        for col, occupied in enumerate(cells):
            x, y = _cell_center(row, col, 21, 21, 2.5)
            hits = sum(abs(x - bx) < sx / 2 and abs(y - by) < sy / 2
                       for bx, by, sx, sy in boxes)
            assert hits == occupied
    assert sum(sx * sy for _, _, sx, sy in boxes) == sum(map(sum, maze)) * 2.5**2
