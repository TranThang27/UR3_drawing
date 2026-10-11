#!/usr/bin/env python3
"""Run and record a real SLAM → map reload → AMCL → three RViz goals demo."""
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import yaml
from capture_x11_window import WindowCapture
from run_demo import session_members

ROOT=Path(__file__).resolve().parents[1]
WM=shutil.which('wmctrl') or str(ROOT/'.runtime/root/usr/bin/wmctrl')

def stop_session(pid):
    for sig,seconds in [(signal.SIGINT,7),(signal.SIGTERM,3),(signal.SIGKILL,1)]:
        members=session_members(pid)
        if not members:return
        for member in members:
            try:os.kill(member,sig)
            except ProcessLookupError:pass
        until=time.monotonic()+seconds
        while session_members(pid) and time.monotonic()<until:time.sleep(.1)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'reports/video_demo')
    args=parser.parse_args();out=args.output.resolve()
    if out.exists():parser.error(f'Output already exists: {out}; choose a new directory')
    out.mkdir(parents=True);(out/'maps').mkdir();(out/'logs').mkdir()
    state={'phase':'Chuẩn bị mô phỏng','detail':'Phòng vuông 6 × 6 m, hai hộp và một trụ tròn'}
    started=None;chapters=[];processes=[];handles=[];video=None
    partition=f'tb4_video_{os.getpid()}'
    env={**os.environ,'IGN_PARTITION':partition,
         'FASTRTPS_DEFAULT_PROFILES_FILE':str(ROOT/'config/fastdds_video_udp.xml'),
         'ROS_LOCALHOST_ONLY':'0'}
    def write_state():
        temporary=out/'state.tmp';temporary.write_text(json.dumps(state,ensure_ascii=False));temporary.replace(out/'state.json')
    def phase(title,detail):
        state.update(phase=title,detail=detail);write_state()
        chapters.append({'title':title,'start_s':time.monotonic()-started if started else 0,'detail':detail})
        (out/'chapters.json').write_text(json.dumps(chapters,indent=2,ensure_ascii=False)+'\n')
        print(title+' — '+detail,flush=True)
    def launch(command,log):
        handle=(out/'logs'/log).open('w');handles.append(handle)
        process=subprocess.Popen([str(v) for v in command],cwd=ROOT,env=env,stdout=handle,
                                 stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(process);return process
    def checked(command,log,timeout=120):
        process=launch(command,log)
        process.wait(timeout=timeout)
        if process.returncode:raise RuntimeError(f'{log} failed: {process.returncode}')
        return process
    def windows():
        text=subprocess.check_output([str(WM),'-l'],text=True)
        result={}
        for line in text.splitlines():
            if line.endswith(' Gazebo'):result['gazebo']=int(line.split()[0],16)
            if line.endswith(' - RViz'):result['rviz']=int(line.split()[0],16)
        return result
    def arrange():
        deadline=time.monotonic()+40
        while time.monotonic()<deadline:
            found=windows()
            if 'gazebo' in found and 'rviz' in found:break
            time.sleep(.4)
        else:raise RuntimeError('Gazebo/RViz windows did not appear')
        for key,geometry in [('gazebo','0,0,100,600,960'),('rviz','0,600,100,1320,960')]:
            subprocess.run([str(WM),'-ir',hex(found[key]),'-b','remove,maximized_vert,maximized_horz'],check=True)
            subprocess.run([str(WM),'-ir',hex(found[key]),'-e',geometry],check=True)
        time.sleep(2)
        state.update(found);write_state()
        result=subprocess.run(['ign','service','-s','/gui/move_to/pose','--reqtype','ignition.msgs.GUICamera',
            '--reptype','ignition.msgs.Boolean','--timeout','3000','--req',
            'pose: {position: {x: 0, y: 0, z: 12}, orientation: {x: -0.5, y: 0.5, z: 0.5, w: 0.5}}'],
            env=env,capture_output=True,text=True,timeout=8)
        if 'data: true' not in result.stdout:raise RuntimeError('Gazebo camera service failed: '+result.stdout+result.stderr)
        return found
    def config(source,name):
        data=yaml.safe_load((ROOT/'config'/source).read_text())
        data['Visualization Manager']['Views']['Current'].update(Scale=110,Angle=0,X=0,Y=0)
        data['Window Geometry'].update(Width=1320,Height=960)
        for display in data['Visualization Manager']['Displays']:
            if display['Class']=='rviz_default_plugins/Path':display['Line Style']='Billboards'
        path=out/name;path.write_text(yaml.safe_dump(data,sort_keys=False));return path
    def simulator():
        process=launch([ROOT/'run.sh','sim','rviz:=false'],'simulation_'+str(len(chapters))+'.log')
        deadline=time.monotonic()+150;attempt=0
        while time.monotonic()<deadline:
            attempt+=1
            check=launch([ROOT/'run.sh','check','--timeout','5','--report',out/'topics.json'],f'topics_{len(chapters)}_{attempt}.log')
            try:check.wait(timeout=60)
            except subprocess.TimeoutExpired:
                stop_session(check.pid)
                continue
            if not check.returncode:return process
            if process.poll() is not None:raise RuntimeError('Simulator exited')
        raise RuntimeError('Simulation topics did not become ready')
    def ready_log(path,marker,process,timeout=70):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if marker in path.read_text(errors='replace'):return
            if process.poll() is not None:raise RuntimeError('Launch exited before activation')
            time.sleep(.3)
        raise RuntimeError('Activation timeout: '+str(path))
    success=False;report={'passed':False,'world':'simple_square','map':str(out/'maps/map.yaml'),
                          'mapping_control':'Automatic fixed survey route with slow velocity commands',
                          'navigation_goal_source':'Actual RViz Nav2 Goal mouse tool',
                          'manual_velocity_after_navigation_goals':0}
    try:
        # Stop only the known project sessions, preserving other desktop apps.
        old=[]
        for path in [ROOT/'reports/part3/simulation.pid',ROOT/'reports/part3/localization.pid',
                     ROOT/'reports/part4/nav2.pid',ROOT/'reports/part4/rviz.pid']:
            if path.exists():
                try:old.append(int(path.read_text().strip()))
                except ValueError:pass
        for path in (ROOT/'reports').glob('video_demo*/runtime_pids.json'):
            for pid in json.loads(path.read_text()):
                try:
                    command=Path(f'/proc/{pid}/cmdline').read_bytes()
                    if str(ROOT).encode() in command:old.append(pid)
                except FileNotFoundError:pass
        for pid in reversed(list(dict.fromkeys(old))):stop_session(pid)
        checked([ROOT/'run.sh','doctor'],'doctor.log',60)
        sim=simulator()
        slam=launch([ROOT/'run.sh','slam'],'slam.log')
        survey_recorder=launch([sys.executable,ROOT/'scripts/record_survey.py','--output',out/'mapping'],'survey_recorder.log')
        mapping_config=config('mapping.rviz','mapping_video.rviz')
        rviz=launch(['rviz2','-d',mapping_config,'--ros-args','-p','use_sim_time:=true'],'rviz_mapping.log')
        gui=arrange();time.sleep(4)
        write_state();started=time.monotonic()
        video=launch([sys.executable,ROOT/'scripts/record_window_video.py','--state',out/'state.json',
                      '--output',out/'turtlebot4_demo.mp4'],'video.log')
        phase('1/5 — Xây dựng bản đồ bằng SLAM Toolbox','Undock → bỏ dock → khảo sát tự động; RViz cập nhật map từ LiDAR và odometry')
        checked([ROOT/'run.sh','undock'],'undock_mapping.log',110)
        checked([ROOT/'run.sh','remove-dock','--partition',partition],'remove_dock_mapping.log',15)
        capture=WindowCapture(gui['rviz']);capture.grab().save(out/'mapping_start.png');capture.close()
        checked([sys.executable,ROOT/'scripts/auto_survey.py'],'auto_survey.log',700)
        shutil.copy2(ROOT/'reports/auto_survey.json',out/'mapping/auto_survey.json')
        survey=json.loads((out/'mapping/auto_survey.json').read_text())
        if not survey['success'] or survey['bump_messages']:raise RuntimeError('Survey did not pass')
        phase('2/5 — Bản đồ hoàn chỉnh trên RViz2',f'Hoàn tất 17 waypoint • quãng đường {survey["distance_m"]:.1f} m • lưu map.yaml và map.pgm')
        time.sleep(6)
        checked([ROOT/'run.sh','save',out/'maps/map'],'save_map.log',45)
        checked([sys.executable,ROOT/'scripts/crop_square_map.py',out/'maps/map.yaml'],'crop_map.log',30)
        checked([sys.executable,ROOT/'scripts/export_grid_map.py',out/'maps/map.yaml'],'grid_map.log',30)
        capture=WindowCapture(gui['rviz']);capture.grab().save(out/'mapping_complete.png');capture.close()
        time.sleep(6)
        phase('3/5 — Load lại bản đồ đã lưu','Dừng SLAM; khởi động lại TurtleBot 4 trong cùng world simple_square')
        state.update(gazebo=None,rviz=None);write_state();time.sleep(.5)
        for process in [survey_recorder,slam,rviz,sim]:stop_session(process.pid)
        sim=simulator()
        navigation_config=config('navigation_report.rviz','navigation_video.rviz')
        rviz=launch(['rviz2','-d',navigation_config,'--ros-args','-p','use_sim_time:=true'],'rviz_navigation.log')
        gui=arrange()
        localize=launch([ROOT/'run.sh','localize',out/'maps/map.yaml'],'localization.log')
        ready_log(out/'logs/localization.log','Managed nodes are active',localize)
        phase('3/5 — Load lại map.yaml','Map server đã load bản đồ vừa tạo; SLAM đã dừng; chuẩn bị đặt initial pose')
        time.sleep(6)
        checked([ROOT/'run.sh','undock'],'undock_localization.log',110)
        checked([ROOT/'run.sh','remove-dock','--partition',partition],'remove_dock_localization.log',15)
        phase('4/5 — Localization bằng AMCL','Initial pose gần (-0.35, 0), hướng -X; kiểm tra vị trí RViz so với Gazebo')
        checked([sys.executable,ROOT/'scripts/check_localization.py','--initial-pose','-.35','0','3.141592653589793',
                 '--map',out/'maps/map.yaml','--report',out/'localization_check.json'],'localization_check.log',180)
        localization=json.loads((out/'localization_check.json').read_text())
        if not localization['passed'] or not localization['slam_stopped']:raise RuntimeError('Localization did not pass')
        sample=localization['samples'][-1]
        phase('4/5 — Localization đã kiểm tra',f'Sai số vị trí {100*sample["position_error_m"]:.1f} cm • hướng {sample["yaw_error_deg"]:.1f}° • LiDAR khớp map')
        capture=WindowCapture(gui['rviz']);capture.grab().save(out/'localization.png');capture.close()
        time.sleep(6)
        nav=launch([ROOT/'run.sh','nav2'],'nav2.log')
        ready_log(out/'logs/nav2.log','Managed nodes are active',nav)
        recorder=launch([sys.executable,ROOT/'scripts/record_navigation.py','--window',hex(gui['rviz']),
                         '--output',out/'navigation'],'navigation_recorder.log')
        time.sleep(5)
        phase('5/5 — Navigation với Nav2','AMCL + inflation + Global Plan; gửi lần lượt 3 goal bằng công cụ Nav2 Goal')
        state['legend']='Đỏ: AMCL/LiDAR • tím/hồng: inflation • xanh lá: Global Plan • cam: đường robot đã đi';write_state()
        for number,(gx,gy,yaw) in enumerate([(1.8,-1.8,0),(-1.8,-1.8,0),(-1.8,2.0,0)],1):
            capture=WindowCapture(gui['rviz']);image=capture.grab();capture.close()
            row=[image.getpixel((px,100)) for px in range(image.width)]
            runs=[];begin=None
            for px,pixel in enumerate(row+[(0,0,0)]):
                if pixel==(48,48,48):
                    if begin is None:begin=px
                elif begin is not None:runs.append((begin,px));begin=None
            left,right=max(runs,key=lambda item:item[1]-item[0])
            center=[(left+right-1)/2,(79+image.height-40)/2]
            phase(f'5/5 — Nav2 Goal {number}/3',f'Đích x={gx:.1f}, y={gy:.1f} m • Nav2 tự lập kế hoạch và điều khiển robot')
            checked([sys.executable,ROOT/'scripts/click_rviz_goal.py',gx,gy,yaw,'--window',hex(gui['rviz']),
                     '--center',*center,'--scale',110,'--log',out/'rviz_goal_clicks.json'],f'goal_{number}_click.log',15)
            deadline=time.monotonic()+150
            while time.monotonic()<deadline:
                file=out/'navigation/navigation_progress.json'
                if file.exists():
                    progress=json.loads(file.read_text());goals=progress['goals']
                    if progress['bump_messages']:raise RuntimeError('Bumper contact during navigation')
                    if len(goals)>=number:
                        goal=goals[number-1]
                        if goal['status'] in (5,6):raise RuntimeError('Navigation canceled/aborted')
                        if goal['status']==4:break
                if recorder.poll() is not None:
                    file=out/'navigation/navigation_results.json'
                    if file.exists():
                        progress=json.loads(file.read_text());goals=progress['goals']
                        if len(goals)>=number and goals[number-1]['status']==4:
                            goal=goals[number-1]
                            break
                    raise RuntimeError('Navigation recorder exited before success')
                time.sleep(.4)
            else:raise RuntimeError('Navigation goal timed out')
            phase(f'5/5 — Goal {number}/3: SUCCEEDED',f'Robot đã tới đích • sai số {goal["position_error_m"]:.2f} m • không có bumper')
            time.sleep(4)
        recorder.wait(timeout=15)
        navigation=json.loads((out/'navigation/navigation_results.json').read_text())
        if not navigation['passed']:raise RuntimeError('Three navigation goals did not pass validation')
        phase('Hoàn thành — SLAM → lưu/load map → AMCL → 3 goal Nav2','3/3 SUCCEEDED • không có tín hiệu bumper • robot tự di chuyển sau mỗi goal')
        time.sleep(8)
        report.update(passed=True,mapping=survey,localization=localization,navigation=navigation,
                      video=str(out/'turtlebot4_demo.mp4'),chapters=chapters)
        success=True
    except (Exception,KeyboardInterrupt) as error:
        message=str(error) or 'Interrupted by user'
        report['error']=message;print('DEMO FAILED: '+message,flush=True)
        phase('Demo bị ngắt — chưa xác nhận hoàn thành',message)
    finally:
        if video:
            state['stop']=True;write_state();video.wait(timeout=40)
        if not success:
            for process in reversed(processes):
                if process.poll() is None:stop_session(process.pid)
        (out/'demo_results.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
        (out/'runtime_pids.json').write_text(json.dumps([p.pid for p in processes if p.poll() is None])+'\n')
        for handle in handles:handle.close()
    return 0 if success else 1

if __name__=='__main__':raise SystemExit(main())
