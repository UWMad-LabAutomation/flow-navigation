# Copyright 2026 Zach Gaier
# Licensed under the Apache License, Version 2.0.

"""Select WSLg hardware OpenGL for Gazebo/RViz without changing system settings."""

import os
from pathlib import Path
import platform


def graphics_environment(environ=None):
    """Use Mesa D3D12 on GPU-capable WSL, respecting explicit user overrides."""
    environ = os.environ if environ is None else environ
    if environ.get('LIBGL_ALWAYS_SOFTWARE', '').lower() in ('1', 'true', 'yes'):
        return {}
    if any(environ.get(key) for key in ('GALLIUM_DRIVER', 'MESA_LOADER_DRIVER_OVERRIDE')):
        return {}
    if ('microsoft' not in platform.release().lower()
            or not Path('/dev/dxg').exists()
            or not Path('/usr/lib/x86_64-linux-gnu/dri/d3d12_dri.so').is_file()):
        return {}
    return {'MESA_LOADER_DRIVER_OVERRIDE': 'd3d12', 'GALLIUM_DRIVER': 'd3d12'}
