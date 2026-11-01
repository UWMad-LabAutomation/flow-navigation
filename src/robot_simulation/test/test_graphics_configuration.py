# Copyright 2026 Zach Gaier
# Licensed under the Apache License, Version 2.0.

"""Check graphics selection across WSL, native Linux and explicit overrides."""

from pathlib import Path

import pytest
from robot_simulation.graphics_configuration import graphics_environment


@pytest.fixture
def wsl_gpu(monkeypatch):
    """Represent a WSL host with the Mesa D3D12 driver and GPU device."""
    monkeypatch.setattr('platform.release', lambda: '6.6.87.2-microsoft-standard-WSL2')
    monkeypatch.setattr(Path, 'exists', lambda self: True)
    monkeypatch.setattr(Path, 'is_file', lambda self: True)


def test_wsl_selects_hardware_graphics(wsl_gpu):
    """Avoid silently rendering both interfaces on CPU when D3D12 is available."""
    assert graphics_environment({}) == {
        'MESA_LOADER_DRIVER_OVERRIDE': 'd3d12', 'GALLIUM_DRIVER': 'd3d12'}


def test_native_linux_keeps_its_driver(wsl_gpu, monkeypatch):
    """Do not force Windows graphics on the deployment target or native Linux."""
    monkeypatch.setattr('platform.release', lambda: '6.8.0-ubuntu')
    assert graphics_environment({}) == {}


@pytest.mark.parametrize('environment', [
    {'LIBGL_ALWAYS_SOFTWARE': 'true'}, {'GALLIUM_DRIVER': 'llvmpipe'},
    {'MESA_LOADER_DRIVER_OVERRIDE': 'zink'},
])
def test_explicit_graphics_choice_is_respected(wsl_gpu, environment):
    """Allow explicit troubleshooting settings instead of overriding the user."""
    assert graphics_environment(environment) == {}


def test_missing_wsl_gpu_does_not_force_d3d12(wsl_gpu, monkeypatch):
    """Leave the default renderer available when the host lacks GPU passthrough."""
    monkeypatch.setattr(Path, 'exists', lambda self: False)
    assert graphics_environment({}) == {}
