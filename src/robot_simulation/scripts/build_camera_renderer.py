#!/usr/bin/env python3
# Copyright 2026 Zach Gaier
# Licensed under the Apache License, Version 2.0.

"""Build a pinned, locally patched Harmonic renderer without changing /usr."""

import argparse
import hashlib
from pathlib import Path
import subprocess
import tarfile
import urllib.request


VERSION = 'gz-rendering8_8.2.3'
ARCHIVE_SHA256 = '5bfe18d164414e098b2ba6ef81bf9391f7ff8f2c39b8a4fa9f8543f778e921c8'


def main():
    """Download, verify, patch, and install only under this workspace."""
    package = Path(__file__).resolve().parents[1]
    workspace = package.parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jobs', type=int, default=2)
    parser.add_argument('--archive', type=Path, help='Use an already downloaded source archive.')
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error('--jobs must be positive')
    work = workspace / 'build/camera_renderer'
    prefix = workspace / 'install/robot_simulation/opt/camera_renderer'
    work.mkdir(parents=True, exist_ok=True)
    archive = args.archive or work / f'{VERSION}.tar.gz'
    if not archive.exists():
        url = f'https://codeload.github.com/gazebosim/gz-rendering/tar.gz/refs/tags/{VERSION}'
        print(f'Downloading {url}', flush=True)
        with urllib.request.urlopen(url, timeout=120) as response:
            archive.write_bytes(response.read())
    if hashlib.sha256(archive.read_bytes()).hexdigest() != ARCHIVE_SHA256:
        raise RuntimeError(f'Source archive checksum mismatch: {archive}')
    source = work / f'gz-rendering-{VERSION}'
    if not source.exists():
        with tarfile.open(archive) as bundle:
            bundle.extractall(work, filter='data')
    patch = package / 'patches/gz-rendering8-depth-projection.patch'
    already_applied = subprocess.run(
        ['patch', '--batch', '--force', '--reverse', '--dry-run', '-p1', '-i', str(patch)],
        cwd=source, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    ).returncode == 0
    if not already_applied:
        subprocess.run(['patch', '--batch', '--forward', '-p1', '-i', str(patch)],
                       cwd=source, check=True)
    subprocess.run(
        ['patch', '--batch', '--force', '--reverse', '--dry-run', '-p1', '-i', str(patch)],
        cwd=source, check=True)
    prefix.mkdir(parents=True, exist_ok=True)
    (prefix / 'flow_projection_fix.txt').write_text('BUILD_IN_PROGRESS\n')
    build = work / 'cmake'
    build.mkdir(parents=True, exist_ok=True)
    subprocess.run([
        'cmake', '-S', str(source), '-B', str(build),
        '-DCMAKE_BUILD_TYPE=Release', f'-DCMAKE_INSTALL_PREFIX={prefix}',
        '-DCMAKE_INSTALL_LIBDIR=lib', '-DBUILD_TESTING=OFF',
        '-DSKIP_ogre=ON', '-DSKIP_optix=ON',
    ], cwd=build, check=True)
    subprocess.run(['cmake', '--build', str(build), '--parallel', str(args.jobs)], check=True)
    subprocess.run(['cmake', '--install', str(build)], check=True)
    (prefix / 'flow_projection_fix.txt').write_text(
        f'{VERSION}\npatch_sha256={hashlib.sha256(patch.read_bytes()).hexdigest()}\n')
    print(f'Camera renderer installed in {prefix}', flush=True)


if __name__ == '__main__':
    main()
