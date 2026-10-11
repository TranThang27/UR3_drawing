#!/usr/bin/env python3
"""One command: simulation, readiness check, undock, and keyboard control."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from save_map import validate_map

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--mapping', action='store_true')
    mode.add_argument('--navigation', action='store_true')
    parser.add_argument('--auto-survey', action='store_true',
                        help='Survey the known simple_square world without keyboard input')
    parser.add_argument('--keep-dock', action='store_true',
                        help='Keep the charging dock after undock (default removes it)')
    parser.add_argument('--map-prefix', type=Path, default=ROOT / 'maps/map')
    parser.add_argument('--map', type=Path, default=ROOT / 'maps/map.yaml')
    options, launch_args = parser.parse_known_args()
    if options.auto_survey and not options.mapping:
        parser.error('--auto-survey requires --mapping.')
    if options.auto_survey and any(arg.startswith('world:=') and arg != 'world:=simple_square'
                                   for arg in launch_args):
        parser.error('Auto survey route supports only world:=simple_square.')
    if options.navigation:
        options.map = options.map.resolve()
        if not options.map.is_file():
            parser.error(f'Chưa có map: {options.map}. Chạy ./run.sh mapping và lưu map trước.')
        try:
            validate_map(options.map)
        except (ValueError, KeyError, OSError) as error:
            parser.error(f'Map không hợp lệ: {error}')
        existing = subprocess.run(['ros2', 'node', 'list'], capture_output=True,
                                  text=True, timeout=15, check=True).stdout.splitlines()
        if any(name in existing for name in ('/slam_toolbox', '/amcl')):
            parser.error('Dừng phiên SLAM/localization cũ bằng Ctrl+C trước khi chạy navigation.')
    reports = ROOT / 'reports'
    reports.mkdir(exist_ok=True)
    simulator = None
    background = []
    log_handles = []
    mapping_ready = False
    exit_code = 0
    with (reports / 'simulation.log').open('w') as log:
        try:
            subprocess.run([str(ROOT / 'run.sh'), 'doctor'], check=True)
            simulator = subprocess.Popen(
                [str(ROOT / 'run.sh'), 'sim', *launch_args,
                 *([f'rviz_config:={ROOT / "config/navigation.rviz"}'] if options.navigation else
                   [f'rviz_config:={ROOT / "config/mapping.rviz"}'] if options.mapping else [])],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                env={**os.environ, 'IGN_PARTITION': f'tb4_demo_{os.getpid()}'})
            print('Đang mở Gazebo/RViz2 và chờ sensor/controller...', flush=True)
            deadline = time.monotonic() + 120
            while time.monotonic() < deadline:
                if simulator.poll() is not None:
                    raise RuntimeError('Simulator đã dừng. Xem reports/simulation.log.')
                checked = subprocess.run(
                    [str(ROOT / 'run.sh'), 'check', '--timeout', '5', '--report',
                     str(reports / ('part2_topics.json' if options.mapping else 'part1_topics.json'))],
                    capture_output=True, text=True)
                if checked.returncode == 0:
                    print(checked.stdout, flush=True)
                    break
                print('Đang chờ dữ liệu hợp lệ...', flush=True)
            else:
                print(checked.stdout)
                raise RuntimeError('Simulator chưa sẵn sàng. Xem reports/simulation.log.')
            if options.mapping:
                for command, filename in [
                    ([str(ROOT / 'run.sh'), 'slam'], 'mapping_slam.log'),
                    ([sys.executable, str(ROOT / 'scripts/record_survey.py'), '--output',
                      str(reports / 'mapping')], 'mapping_survey.log'),
                ]:
                    handle = (reports / filename).open('w')
                    log_handles.append(handle)
                    background.append(subprocess.Popen(
                        command, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True))
                mapped = subprocess.run(
                    ['ros2', 'topic', 'echo', '/map', 'nav_msgs/msg/OccupancyGrid',
                     '--once', '--field', 'info',
                     '--qos-durability', 'transient_local'], timeout=30, capture_output=True, text=True)
                if mapped.returncode != 0 or not mapped.stdout.strip():
                    if mapped.stderr.strip():
                        print(mapped.stderr, file=sys.stderr)
                    raise RuntimeError('SLAM chưa publish map. Xem reports/mapping_slam.log.')
                print('SLAM đang chạy; theo dõi Map và Survey Path trên RViz2.', flush=True)
            subprocess.run([str(ROOT / 'run.sh'), 'undock'], check=True)
            if not options.keep_dock:
                world = next((arg.split(':=',1)[1] for arg in launch_args
                              if arg.startswith('world:=')), 'simple_square')
                subprocess.run([sys.executable, str(ROOT/'scripts/remove_dock.py'),
                                '--world',world,'--partition',f'tb4_demo_{os.getpid()}'],check=True)
            mapping_ready = options.mapping
            if options.navigation:
                handle = (reports / 'navigation.log').open('w')
                log_handles.append(handle)
                navigation = subprocess.Popen(
                    [str(ROOT / 'run.sh'), 'nav', str(options.map)],
                    stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
                background.append(navigation)
                print('AMCL/Nav2 đang khởi động. RViz2: 2D Pose Estimate → kiểm tra LiDAR '
                      'khớp map → Nav2 Goal. Ctrl+C để đóng.', flush=True)
                while simulator.poll() is None and navigation.poll() is None:
                    time.sleep(0.5)
                raise RuntimeError('Simulator hoặc Nav2 đã dừng; xem reports/navigation.log.')
            if options.auto_survey:
                # A failed/incomplete survey must not replace the final map.
                mapping_ready = False
                subprocess.run([sys.executable, str(ROOT / 'scripts/auto_survey.py')], check=True)
                mapping_ready = True
            else:
                print('Sẵn sàng: i tiến, j/l quay, k dừng.', flush=True)
                print('Ctrl+C kết thúc khảo sát và lưu map.yaml/map.pgm.' if options.mapping
                      else 'Ctrl+C để đóng demo.', flush=True)
                subprocess.run([str(ROOT / 'run.sh'), 'teleop'], check=True)
        except KeyboardInterrupt:
            exit_code = 0 if mapping_ready or options.navigation else 130
        except (RuntimeError, subprocess.SubprocessError) as error:
            print(error, file=sys.stderr)
            exit_code = 1
        finally:
            if mapping_ready and simulator.poll() is not None:
                print('Simulator đã dừng trước khi lưu; map chưa được xác nhận.', file=sys.stderr)
                exit_code = 1
            if mapping_ready and simulator.poll() is None:
                print('Đang lưu bản đồ trước khi đóng SLAM...', flush=True)
                try:
                    saved = subprocess.run([str(ROOT / 'run.sh'), 'save', str(options.map_prefix)])
                    if saved.returncode:
                        exit_code = 1
                    elif options.auto_survey:
                        subprocess.run([sys.executable, str(ROOT / 'scripts/crop_square_map.py'),
                                        str(options.map_prefix) + '.yaml'], check=True)
                        subprocess.run([sys.executable, str(ROOT / 'scripts/export_grid_map.py'),
                                        str(options.map_prefix) + '.yaml'], check=True)
                except KeyboardInterrupt:
                    print('Lưu map bị ngắt; không coi bản đồ là đã lưu.', file=sys.stderr)
                    exit_code = 130
            for process in reversed(background):
                stop_session(process)
            if simulator is not None:
                stop_session(simulator)
            for handle in log_handles:
                handle.close()
    return exit_code


def stop_session(process):
    # Gazebo creates separate process groups within the launcher's session.
    for sig, seconds in [(signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 1)]:
        members = session_members(process.pid)
        if not members:
            break
        for pid in members:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        until = time.monotonic() + seconds
        while session_members(process.pid) and time.monotonic() < until:
            time.sleep(0.1)
    process.wait()


def session_members(session_id):
    processes = subprocess.check_output(['ps', '-eo', 'pid=,sid=,stat='], text=True)
    return [int(pid) for pid, sid, status in (line.split() for line in processes.splitlines())
            if int(sid) == session_id and not status.startswith('Z')]


if __name__ == '__main__':
    raise SystemExit(main())
