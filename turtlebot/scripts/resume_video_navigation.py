#!/usr/bin/env python3
"""Continue an interrupted video after successful mapping and localization."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
from capture_x11_window import WindowCapture
from run_video_demo import ROOT,stop_session

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder',type=Path)
    parser.add_argument('--goal-source',choices=['rviz','ros_action'],required=True)
    args=parser.parse_args();out=args.folder.resolve()
    old=json.loads((out/'demo_results.json').read_text())
    local=json.loads((out/'localization_check.json').read_text())
    survey=json.loads((out/'mapping/auto_survey.json').read_text())
    if not local['passed'] or not local['slam_stopped'] or not survey['success']:
        raise RuntimeError('Successful mapping and localization are required')
    chapters=json.loads((out/'chapters.json').read_text())
    cutoff=next(item['start_s'] for item in chapters if item['title'].startswith('5/5'))
    chapters=[item for item in chapters if item['start_s']<cutoff]
    for pid in json.loads((out/'runtime_pids.json').read_text()):
        try:
            command=Path(f'/proc/{pid}/cmdline').read_bytes()
            if b'record_navigation.py' in command:stop_session(pid)
        except FileNotFoundError:pass
    if (out/'navigation').exists():
        (out/'navigation').rename(out/'navigation_locked_attempt')
    raw=out/'turtlebot4_before_navigation.mp4'
    (out/'turtlebot4_demo.mp4').rename(raw)
    stats=out/'turtlebot4_demo.recording.json'
    if stats.exists():stats.rename(raw.with_suffix('.recording.json'))
    shutil.copy2(out/'demo_results.json',out/'interrupted_results.json')
    state=json.loads((out/'state.json').read_text());state.pop('stop',None)
    env={**os.environ,'FASTRTPS_DEFAULT_PROFILES_FILE':str(ROOT/'config/fastdds_video_udp.xml'),
         'ROS_LOCALHOST_ONLY':'0'}
    logs=[];processes=[];start=time.monotonic()
    def phase(title,detail):
        state.update(phase=title,detail=detail)
        temp=out/'state.tmp';temp.write_text(json.dumps(state,ensure_ascii=False));temp.replace(out/'state.json')
        chapters.append({'title':title,'start_s':cutoff+time.monotonic()-start,'detail':detail})
        print(title+' — '+detail,flush=True)
    def launch(command,name):
        log=(out/'logs'/name).open('w');logs.append(log)
        p=subprocess.Popen([str(v) for v in command],cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(p);return p
    source='công cụ Nav2 Goal trên RViz' if args.goal_source=='rviz' else 'ROS2 NavigateToPose action (màn hình máy đang khóa)'
    phase('5/5 — Navigation với Nav2',f'Gửi goal bằng {source}; robot tự lập kế hoạch và di chuyển')
    recorder=launch([sys.executable,ROOT/'scripts/record_navigation.py','--window',hex(state['rviz']),
                     '--output',out/'navigation','--goal-source',args.goal_source],'navigation_resumed.log')
    video=launch([sys.executable,ROOT/'scripts/record_window_video.py','--state',out/'state.json',
                  '--output',out/'turtlebot4_navigation_segment.mp4'],'video_navigation.log')
    success=False
    try:
        time.sleep(5)
        for number,(gx,gy,yaw) in enumerate([(1.8,-1.8,0),(-1.8,-1.8,0),(-1.8,2.0,0)],1):
            phase(f'5/5 — Nav2 Goal {number}/3',f'Đích ({gx:.1f}, {gy:.1f}) m • {source}')
            if args.goal_source=='ros_action':
                sender=launch([sys.executable,ROOT/'scripts/check_navigation.py','--goal',gx,gy,yaw,
                    '--timeout','150','--report',out/f'action_goal_{number}.json'],f'action_goal_{number}.log')
                sender.wait(timeout=200)
                if sender.returncode:raise RuntimeError(f'Action goal {number} failed')
            else:
                capture=WindowCapture(state['rviz']);image=capture.grab();capture.close()
                runs=[];begin=None
                for px in range(image.width+1):
                    pixel=image.getpixel((px,100)) if px<image.width else (0,0,0)
                    if pixel==(48,48,48):
                        if begin is None:begin=px
                    elif begin is not None:runs.append((begin,px));begin=None
                left,right=max(runs,key=lambda pair:pair[1]-pair[0])
                sender=launch([sys.executable,ROOT/'scripts/click_rviz_goal.py',gx,gy,yaw,
                    '--window',hex(state['rviz']),'--center',(left+right-1)/2,(79+image.height-40)/2,
                    '--scale','110','--log',out/'rviz_goal_clicks.json'],f'gui_goal_{number}.log')
                sender.wait(timeout=15)
                if sender.returncode:raise RuntimeError('RViz click failed: unlock the desktop')
            deadline=time.monotonic()+150
            while time.monotonic()<deadline:
                file=out/'navigation/navigation_progress.json'
                progress=json.loads(file.read_text())
                if len(progress['goals'])>=number:
                    goal=progress['goals'][number-1]
                    if goal['status'] in (5,6) or progress['bump_messages']:raise RuntimeError('Navigation failed/contact')
                    if goal['status']==4:break
                if recorder.poll() is not None:
                    progress=json.loads((out/'navigation/navigation_results.json').read_text())
                    goal=progress['goals'][number-1]
                    if goal['status']==4:break
                    raise RuntimeError('Navigation recorder ended before success')
                time.sleep(.3)
            else:raise RuntimeError('Goal timeout')
            phase(f'5/5 — Goal {number}/3: SUCCEEDED',f'Robot tới đích • sai số {goal["position_error_m"]:.2f} m • không bumper')
            time.sleep(4)
        recorder.wait(timeout=15)
        navigation=json.loads((out/'navigation/navigation_results.json').read_text())
        if not navigation['passed']:raise RuntimeError('Navigation validation failed')
        phase('Hoàn thành — SLAM → lưu/load map → AMCL → 3 goal Nav2','3/3 SUCCEEDED • robot tự di chuyển • không bumper')
        time.sleep(8);success=True
    finally:
        state['stop']=True
        temp=out/'state.tmp';temp.write_text(json.dumps(state,ensure_ascii=False));temp.replace(out/'state.json')
        video.wait(timeout=40)
        if recorder.poll() is None:stop_session(recorder.pid)
        for log in logs:log.close()
    if not success:return 1
    # Preserve the original interrupted capture. Remove only the locked idle tail.
    filters=f'[0:v]trim=end={cutoff:.3f},setpts=PTS-STARTPTS[a];[1:v]setpts=PTS-STARTPTS[b];[a][b]concat=n=2:v=1:a=0[out]'
    subprocess.run(['ffmpeg','-y','-hide_banner','-loglevel','warning','-i',str(raw),
        '-i',str(out/'turtlebot4_navigation_segment.mp4'),'-filter_complex',filters,'-map','[out]',
        '-an','-c:v','libx264','-preset','veryfast','-crf','20','-pix_fmt','yuv420p',
        '-movflags','+faststart',str(out/'turtlebot4_demo.mp4')],check=True)
    old.update(passed=True,mapping=survey,localization=local,navigation=navigation,chapters=chapters,
               navigation_goal_source=source,video=str(out/'turtlebot4_demo.mp4'),
               editing_note='Joined actual mapping/localization capture with actual navigation continuation; removed idle locked-screen waiting. Original captures retained.')
    old.pop('error',None)
    (out/'demo_results.json').write_text(json.dumps(old,indent=2,ensure_ascii=False)+'\n')
    (out/'chapters.json').write_text(json.dumps(chapters,indent=2,ensure_ascii=False)+'\n')
    return 0

if __name__=='__main__':raise SystemExit(main())
