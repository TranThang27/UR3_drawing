#!/usr/bin/python3
"""Extract verified Ubuntu/ROS debs into a private runtime, without sudo."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import urllib.request
from zipfile import ZipFile

import apt


ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / '.runtime'
PACKAGES = [
    'ros-humble-turtlebot4-simulator', 'ros-humble-irobot-create-nodes',
    'ros-humble-turtlebot4-navigation', 'ros-humble-turtlebot4-viz',
    'ros-humble-slam-toolbox', 'ros-humble-nav2-bringup',
    'ros-humble-irobot-create-description',
]


def configure_sensor_renderer(ros_prefix):
    """Use the backend verified against stationary SDF ray intersections."""
    description = ros_prefix / 'share/irobot_create_description/urdf/create3.urdf.xacro'
    original = description.read_text()
    old, new = '<render_engine>ogre</render_engine>', '<render_engine>ogre2</render_engine>'
    if old not in original and new not in original:
        raise RuntimeError('Unexpected Create 3 sensor renderer configuration')
    description.write_text(original.replace(old, new))
    (RUNTIME / 'sensor_renderer.json').write_text(json.dumps({
        'file': str(description.relative_to(RUNTIME)),
        'render_engine': 'ogre2',
        'reason': 'Ogre stationary GPU LiDAR produced localized wall distortions on this machine; Ogre2 removed the wall overshoot in the A/B test.',
        'evidence': 'reports/lidar_investigation/stationary.json and ogre2.json',
        'scope': 'Create 3 Sensors system, including LiDAR and IR sensors; GUI engine is configured separately.',
    }, indent=2) + '\n')


def main():
    # Resolve in simulation only. The LLVM conflict is irrelevant in a private
    # extraction tree: no host packages are installed, upgraded or removed.
    plan = subprocess.run([
        'apt-get', '--simulate', '--no-install-recommends', 'install',
        *PACKAGES, 'libunwind-dev', 'libunwind-14-dev-',
    ], check=True, text=True, capture_output=True).stdout
    names = re.findall(r'^Inst (\S+)', plan, flags=re.MULTILINE)
    cache = apt.Cache()
    # Include requested packages even when already installed on the host.
    names = list(dict.fromkeys(names + PACKAGES))
    downloads = RUNTIME / 'downloads'
    downloads.mkdir(parents=True, exist_ok=True)

    def download(name):
        candidate = cache[name].candidate
        target = downloads / Path(candidate.filename).name
        digest = candidate.record['SHA256']
        if not target.exists() or hashlib.sha256(target.read_bytes()).hexdigest() != digest:
            # ROS apt endpoint serves HTTP; integrity is checked against the
            # SHA256 in the system's apt index. Ubuntu supports HTTPS directly.
            uri = candidate.uri
            if 'archive.ubuntu.com' in uri:
                uri = uri.replace('http://', 'https://', 1)
                uri = uri.replace('vn.archive.ubuntu.com', 'archive.ubuntu.com')
            partial = target.with_suffix('.partial')
            with urllib.request.urlopen(uri, timeout=90) as response, partial.open('wb') as out:
                shutil.copyfileobj(response, out)
            if hashlib.sha256(partial.read_bytes()).hexdigest() != digest:
                raise RuntimeError(f'SHA256 mismatch: {name}')
            partial.replace(target)
        print(f'Downloaded and verified {name}', flush=True)
        return {'package': name, 'version': candidate.version, 'sha256': digest,
                'archive': str(target), 'source': candidate.uri}

    with ThreadPoolExecutor(max_workers=4) as pool:
        records = list(pool.map(download, names))
    prefix = RUNTIME / 'root'
    prefix.mkdir(exist_ok=True)
    for record in records:
        subprocess.run(['dpkg-deb', '--extract', record['archive'], str(prefix)], check=True)
    ros_prefix = prefix / 'opt/ros/humble'
    for name in ['local_setup.bash', 'local_setup.sh', '_local_setup_util.py']:
        shutil.copy2(Path('/opt/ros/humble') / name, ros_prefix / name)
    configure_sensor_renderer(ros_prefix)
    # Preload the official Depot world so simulator startup does not depend on
    # Fuel resolving remote assets. Only its include path is changed.
    archive = downloads / 'Depot.zip'
    if not archive.exists():
        with urllib.request.urlopen(
            'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Depot/6/Depot.zip',
            timeout=90,
        ) as response, archive.with_suffix('.partial').open('wb') as out:
            shutil.copyfileobj(response, out)
        archive.with_suffix('.partial').replace(archive)
    model = RUNTIME / 'models/Depot'
    model.mkdir(parents=True, exist_ok=True)
    with ZipFile(archive) as zipped:
        for member in zipped.infolist():
            destination = (model / member.filename).resolve()
            if not destination.is_relative_to(model.resolve()):
                raise RuntimeError('Unsafe model archive path')
        zipped.extractall(model)
    world = ros_prefix / 'share/turtlebot4_ignition_bringup/worlds/depot.sdf'
    world.write_text(world.read_text().replace(
        'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Depot', str(model)))
    (RUNTIME / 'world.json').write_text(json.dumps({
        'name': 'Depot', 'owner': 'OpenRobotics', 'version': 6,
        'source': 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Depot',
        'license': 'CC BY 4.0', 'archive_sha256': hashlib.sha256(archive.read_bytes()).hexdigest(),
    }, indent=2) + '\n')
    (RUNTIME / 'manifest.json').write_text(json.dumps(records, indent=2) + '\n')
    print(f'Runtime ready: {ros_prefix}')


if __name__ == '__main__':
    main()
