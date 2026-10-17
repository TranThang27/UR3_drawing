#!/usr/bin/env python3
"""Load the existing map, validate AMCL, run three code goals with live YOLO, record."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import yaml
from capture_x11_window import WindowCapture
from run_video_demo import stop_session

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'reports' / datetime.now().strftime('nav_yolo_demo_%Y%m%d_%H%M%S'))
    parser.add_argument('--map', type=Path, default=ROOT / 'maps/map.yaml')
    parser.add_argument('--goals-file', type=Path, default=ROOT / 'config/goals.yaml')
    parser.add_argument('--goal-timeout', type=float, default=360)
    args = parser.parse_args()
    out = args.output.resolve()
    if out.exists():
        parser.error('Output exists; choose a new directory.')
    if not (ROOT / '.runtime/yolo-venv/bin/python').exists():
        parser.error('Run ./run.sh setup-yolo first.')
    map_path = args.map.resolve()
    metadata = yaml.safe_load(map_path.read_text())
    image_path = map_path.parent / metadata['image']
    goals = yaml.safe_load(args.goals_file.read_text())
    if goals.get('frame_id') != 'map' or len(goals.get('goals', [])) < 3:
        parser.error('At least three goals in map frame are required.')
    out.mkdir(parents=True); (out / 'logs').mkdir()
    print(f'Output: {out}', flush=True)
    env = dict(os.environ)
    for key in list(env):
        if key.startswith(('SNAP', 'GTK', 'QT', 'GIO', '__NV', '__GLX')):
            env.pop(key, None)
    for key in ['XDG_DATA_DIRS', 'XDG_CONFIG_DIRS']:
        original = env.get(key + '_VSCODE_SNAP_ORIG')
        if original:
            env[key] = original
    env.update(IGN_PARTITION=f'tb4_yolo_{os.getpid()}', ROS_LOCALHOST_ONLY='0',
               RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
               CYCLONEDDS_URI='file://' + str(ROOT / 'config/cyclonedds_local.xml'))
    env.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
    wm = shutil.which('wmctrl') or str(ROOT / '.runtime/root/usr/bin/wmctrl')
    state = {'phase': '1/3 — Khởi động TurtleBot 4, load map đã lưu',
             'detail': 'World simple_square từ tuần trước; thêm ghế và người ở mép phòng'}
    processes = []; handles = []; chapters = []; video = None; started = None
    report = {'passed': False, 'world': 'simple_square', 'map': str(map_path),
              'map_sha256_before': hashlib.sha256(map_path.read_bytes()).hexdigest(),
              'pgm_sha256_before': hashlib.sha256(image_path.read_bytes()).hexdigest(),
              'goals_config': goals, 'manual_velocity_commands_during_mission': 0,
              'navigation_goal_source': 'Python rclpy ActionClient /navigate_to_pose'}

    def save_state():
        temp = out / 'state.tmp'; temp.write_text(json.dumps(state, ensure_ascii=False))
        temp.replace(out / 'state.json')

    def phase(title, detail):
        state.update(phase=title, detail=detail); save_state()
        chapters.append({'title': title, 'start_s': time.monotonic()-started if started else 0,
                         'detail': detail})
        (out / 'chapters.json').write_text(json.dumps(chapters, indent=2, ensure_ascii=False)+'\n')
        print(title, detail, flush=True)

    def launch(command, name):
        handle = (out / 'logs' / name).open('w'); handles.append(handle)
        process = subprocess.Popen([str(v) for v in command], cwd=ROOT, env=env,
            stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        (out / 'runtime_pids.json').write_text(json.dumps([p.pid for p in processes])+'\n')
        return process

    def checked(command, name, timeout):
        process = launch(command, name)
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            stop_session(process.pid)
            raise RuntimeError(f'{name} timed out')
        if process.returncode:
            raise RuntimeError(f'{name} failed; inspect logs')
        return process

    def wait_log(process, name, marker, timeout=120):
        path = out / 'logs' / name; deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            if marker in path.read_text(errors='replace'):
                return
            if process.poll() is not None:
                raise RuntimeError(f'{name} exited before readiness')
            time.sleep(.3)
        raise RuntimeError(f'{name} did not become ready')

    def find_windows(keys):
        deadline = time.monotonic()+60
        while time.monotonic() < deadline:
            found = {}
            for line in subprocess.check_output([wm, '-l'], text=True).splitlines():
                if line.endswith(' Gazebo'): found['gazebo'] = int(line.split()[0], 16)
                if line.endswith(' - RViz'): found['rviz'] = int(line.split()[0], 16)
                if line.endswith(' TurtleBot 4 - YOLO camera'): found['yolo'] = int(line.split()[0], 16)
            if all(k in found for k in keys):
                return found
            time.sleep(.4)
        raise RuntimeError('Expected GUI windows did not appear')

    def snapshot(name):
        capture = WindowCapture(state['rviz']); capture.grab().save(out / f'{name}_rviz.png'); capture.close()
        if state.get('yolo'):
            capture = WindowCapture(state['yolo']); capture.grab().save(out / f'{name}_camera.png'); capture.close()

    try:
        # Only terminate sessions recorded by this project, verifying PID ownership.
        files = list((ROOT / 'reports/yolo_setup').glob('*.pid'))
        files += list((ROOT / 'reports').glob('nav_yolo*/runtime_pids.json'))
        old = []
        for path in files:
            values = json.loads(path.read_text()) if path.suffix == '.json' else [int(path.read_text())]
            for pid in values:
                try:
                    command = Path(f'/proc/{pid}/cmdline').read_bytes()
                    if str(ROOT).encode() in command: old.append(pid)
                except FileNotFoundError:
                    pass
        for pid in reversed(list(dict.fromkeys(old))): stop_session(pid)
        checked([ROOT / 'run.sh', 'doctor'], 'doctor.log', 60)
        sim = launch([ROOT / 'run.sh', 'sim', 'rviz:=false'], 'simulation.log')
        ready = False
        for attempt in range(10):
            check = launch([ROOT / 'run.sh', 'check', '--timeout', '5', '--report', out / 'topics.json'],
                           f'topics_{attempt}.log')
            check.wait(timeout=45)
            if check.returncode == 0:
                ready = True; break
            if sim.poll() is not None:
                raise RuntimeError('Simulator exited')
        if not ready: raise RuntimeError('Simulation sensors did not pass')
        checked([ROOT / 'run.sh', 'undock'], 'undock.log', 120)
        checked([ROOT / 'run.sh', 'remove-dock'], 'remove_dock.log', 20)
        checked([sys.executable, ROOT / 'scripts/add_demo_objects.py', '--output', out / 'objects'], 'objects.log', 60)
        data = yaml.safe_load((ROOT / 'config/navigation_report.rviz').read_text())
        data['Window Geometry'].update(Width=1040, Height=950)
        data['Visualization Manager']['Views']['Current'].update(Scale=90, Angle=0, X=0, Y=0)
        for display in data['Visualization Manager']['Displays']:
            if display['Class'] == 'rviz_default_plugins/Path': display['Line Style'] = 'Billboards'
        cfg = out / 'navigation.rviz'; cfg.write_text(yaml.safe_dump(data, sort_keys=False))
        rviz = launch(['rviz2', '-d', cfg, '--ros-args', '-p', 'use_sim_time:=true'], 'rviz.log')
        detector = launch([ROOT / 'run.sh', 'yolo', '--display', '--output', out / 'yolo'], 'yolo.log')
        wait_log(detector, 'yolo.log', 'YOLO ready')
        state.update(find_windows(['gazebo', 'rviz', 'yolo']))
        for key, geometry in [('gazebo', '0,0,120,880,460'), ('yolo', '0,0,610,640,450'),
                              ('rviz', '0,880,110,1040,950')]:
            subprocess.run([wm, '-ir', hex(state[key]), '-b', 'remove,maximized_vert,maximized_horz'], check=True)
            subprocess.run([wm, '-ir', hex(state[key]), '-e', geometry], check=True)
        subprocess.run(['ign', 'service', '-s', '/gui/move_to/pose', '--reqtype', 'ignition.msgs.GUICamera',
            '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000', '--req',
            'pose: {position: {x: 0, y: 0, z: 6}, orientation: {x: -0.5, y: 0.5, z: 0.5, w: 0.5}}'],
            env=env, capture_output=True, text=True, timeout=8)
        save_state(); started = time.monotonic()
        video = launch([sys.executable, ROOT / 'scripts/record_yolo_video.py', '--state', out / 'state.json',
                        '--output', out / 'turtlebot4_nav_yolo.mp4'], 'video.log')
        localize = launch([ROOT / 'run.sh', 'localize', map_path], 'localization.log')
        wait_log(localize, 'localization.log', 'Managed nodes are active')
        phase('1/3 — Load map từ tuần trước', 'Map server + AMCL; camera YOLO đang nhận ảnh RGB của robot')
        time.sleep(5)
        phase('1/3 — Thiết lập initial pose và kiểm chứng AMCL', 'Initial pose (-0.35, 0, pi); so sánh TF map/base_link với pose Gazebo')
        checked([sys.executable, ROOT / 'scripts/check_localization.py', '--stationary', '--initial-pose',
                 '-.35', '0', '3.141592653589793', '--map', map_path, '--report', out / 'localization.json'],
                'localization_check.log', 150)
        localization = json.loads((out / 'localization.json').read_text())
        if not localization['passed'] or not localization['slam_stopped']:
            raise RuntimeError('Localization validation failed')
        sample = localization['samples'][-1]
        phase('1/3 — Localization đạt', f'Sai số {sample["position_error_m"]*100:.1f} cm; hướng {sample["yaw_error_deg"]:.1f}°; LiDAR khớp tường')
        snapshot('localization'); time.sleep(5)
        nav = launch([ROOT / 'run.sh', 'nav2'], 'nav2.log')
        wait_log(nav, 'nav2.log', 'Managed nodes are active')
        trail = launch([sys.executable, ROOT / 'scripts/record_navigation.py', '--window', hex(state['rviz']),
            '--goals', len(goals['goals']), '--duration', args.goal_timeout*len(goals['goals'])+60,
            '--goal-source', 'ros_action', '--output', out / 'navigation_monitor'], 'navigation_monitor.log')
        phase('2/3 — Node Python gửi Goal 1 → Goal 2 → Goal 3', 'Frame map; yaw theo config/goals.yaml; không teleop, không đặt goal bằng chuột')
        mission = launch([ROOT / 'run.sh', 'goals', '--goals-file', args.goals_file.resolve(),
            '--timeout', args.goal_timeout, '--dwell', '6', '--report', out / 'navigation.json'], 'mission.log')
        last_goal = None
        while mission.poll() is None:
            if detector.poll() is not None: raise RuntimeError('YOLO exited during navigation')
            if nav.poll() is not None or sim.poll() is not None: raise RuntimeError('Simulation/Nav2 exited')
            path = out / 'navigation_progress.json'
            if path.exists():
                progress = json.loads(path.read_text())
                entries = progress['goals']
                current = (len(entries), entries[-1].get('status')) if entries else None
                if current and current != last_goal:
                    done = sum(g.get('succeeded', False) for g in entries)
                    target = entries[-1]['goal']
                    text = 'SUCCEEDED' if entries[-1].get('succeeded') else 'đang di chuyển'
                    phase(f'2/3 — Goal {len(entries)}/{len(goals["goals"])}: {text}',
                          f'Đích ({target[0]}, {target[1]}, yaw={target[2]}); {done} goal hoàn thành; YOLO chạy song song')
                    last_goal = current
            time.sleep(.4)
        if mission.returncode: raise RuntimeError('Navigation mission did not pass')
        navigation = json.loads((out / 'navigation.json').read_text())
        if not navigation['passed'] or navigation['observations']['bump_messages']:
            raise RuntimeError('Navigation result/bumper validation failed')
        checked([sys.executable, ROOT / 'scripts/check_pose_alignment.py', '--report', out / 'final_alignment.json'], 'final_alignment.log', 40)
        report['final_alignment'] = json.loads((out / 'final_alignment.json').read_text())
        phase('3/3 — Hoàn thành 3 goal liên tiếp + YOLO', '3/3 SUCCEEDED; không bumper; camera hiển thị bounding box, class và confidence')
        time.sleep(8)
        summary = json.loads((out / 'yolo/summary.json').read_text())
        if summary['frames_processed'] < 20 or summary['frames_with_detections'] < 10:
            raise RuntimeError('Too few real YOLO frames/detections')
        report.update(localization=localization, navigation=navigation, yolo=summary,
                      map_sha256_after=hashlib.sha256(map_path.read_bytes()).hexdigest(),
                      pgm_sha256_after=hashlib.sha256(image_path.read_bytes()).hexdigest())
        if report['map_sha256_before'] != report['map_sha256_after'] or report['pgm_sha256_before'] != report['pgm_sha256_after']:
            raise RuntimeError('Original map was changed during demo')
        snapshot('completed')
        report['passed'] = True
    except (Exception, KeyboardInterrupt) as error:
        report['error'] = str(error) or 'Interrupted'; print('DEMO FAILED:', report['error'], flush=True)
    finally:
        (out / 'demo_results.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
        if video and video.poll() is None:
            state['stop'] = True; save_state(); video.wait(timeout=60)
        if not report['passed']:
            for process in reversed(processes):
                if process.poll() is None: stop_session(process.pid)
        (out / 'demo_results.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
        (out / 'runtime_pids.json').write_text(json.dumps([p.pid for p in processes if p.poll() is None])+'\n')
        for handle in handles: handle.close()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
