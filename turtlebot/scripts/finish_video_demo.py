#!/usr/bin/env python3
"""Validate a completed demo MP4 and make a labelled shorter version."""
import argparse
import json
from pathlib import Path
import subprocess

def probe(path):
    return json.loads(subprocess.check_output(['ffprobe','-v','error','-show_format',
        '-show_streams','-of','json',str(path)],text=True))

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder',type=Path)
    args=parser.parse_args();folder=args.folder.resolve()
    results=json.loads((folder/'demo_results.json').read_text())
    if not results['passed']:raise RuntimeError('Cannot export a completed demo from a failed run')
    source=folder/'turtlebot4_demo.mp4';info=probe(source)
    video=next(stream for stream in info['streams'] if stream['codec_type']=='video')
    if video['codec_name']!='h264' or video['width']!=1920 or video['height']!=1080:
        raise RuntimeError('Unexpected video format')
    duration=float(info['format']['duration']);chapters=results['chapters']
    boundaries=[
        (0,1,'Khảo sát SLAM: tua nhanh 4x'),
        (next(c['start_s'] for c in chapters if c['title'].startswith('2/5')),1,'Bản đồ hoàn chỉnh: tốc độ thật'),
        (next(c['start_s'] for c in chapters if c['title'].startswith('3/5')),4,'Khởi động lại: tua nhanh 4x'),
        (next(c['start_s'] for c in chapters if c['title']=='3/5 — Load lại map.yaml'),1,'Load bản đồ: tốc độ thật'),
        (next(c['start_s'] for c in chapters if c['title']=='4/5 — Localization bằng AMCL'),2,'Localization: tua nhanh 2x'),
        (next(c['start_s'] for c in chapters if c['title']=='4/5 — Localization đã kiểm tra'),1,'Kiểm tra localization: tốc độ thật'),
        (next(c['start_s'] for c in chapters if c['title'].startswith('5/5')),1,'Nav2: tốc độ thật — 3 goal thành công')]
    boundaries[0]=(0,4,boundaries[0][2])
    filters=[];segments=[];short_chapters=[];elapsed=0
    for index,(start,speed,label) in enumerate(boundaries):
        end=boundaries[index+1][0] if index+1<len(boundaries) else duration
        if end<=start:continue
        # Labels are fixed strings, not shell commands. The source is always real footage.
        output=f'v{index}'
        escaped_label=label.replace(':',r'\:')
        filters.append(f'[0:v]trim=start={start:.3f}:end={end:.3f},setpts=(PTS-STARTPTS)/{speed},fps=10,'
            'drawbox=x=1350:y=5:w=555:h=35:color=black@0.8:t=fill,'
            'drawtext=fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf:'
            f'text=\'{escaped_label}\':x=1360:y=12:fontsize=18:fontcolor=white[{output}]')
        segments.append(f'[{output}]')
        short_chapters.append({'start_s':elapsed,'source_start_s':start,'speed':speed,'label':label})
        elapsed+=(end-start)/speed
    filters.append(''.join(segments)+f'concat=n={len(segments)}:v=1:a=0[out]')
    script=folder/'short_video_filters.txt';script.write_text(';\n'.join(filters)+'\n')
    short=folder/'turtlebot4_demo_short.mp4'
    subprocess.run(['ffmpeg','-y','-hide_banner','-loglevel','warning','-i',str(source),
        '-filter_complex_script',str(script),'-map','[out]','-an','-c:v','libx264',
        '-preset','veryfast','-crf','21','-pix_fmt','yuv420p','-movflags','+faststart',str(short)],check=True)
    short_info=probe(short)
    (folder/'video_info.json').write_text(json.dumps({'full':info,'short':short_info,'short_chapters':short_chapters},indent=2)+'\n')
    navigation=results['navigation'];local=results['localization']['samples'][-1]
    table='\n'.join(f'| {g["number"]} | ({g["target_map_xy"][0]:.2f}, {g["target_map_xy"][1]:.2f}) | SUCCEEDED | {g["position_error_m"]:.3f} m |' for g in navigation['goals'])
    times='\n'.join(f'- {int(c["start_s"]//60):02d}:{int(c["start_s"]%60):02d} — {c["label"]}' for c in short_chapters)
    (folder/'README.md').write_text(f'''# Video demo TurtleBot 4

Lượt chạy thực tế trong Gazebo Fortress, ROS 2 Humble, SLAM Toolbox, AMCL và Nav2.
Video ghép hai cửa sổ Gazebo/RViz đang chạy, có nhãn bước; không dựng lại cảnh.

- `turtlebot4_demo_short.mp4`: bản rút gọn, {float(short_info['format']['duration']):.1f} giây, H.264 1920 × 1080, 10 fps.
- `turtlebot4_demo.mp4`: bản đầy đủ, {duration:.1f} giây, cùng định dạng.
- `maps/map.yaml` và `maps/map.pgm`: bản đồ mới tạo trong chính lượt quay này.
- `demo_results.json`: kết quả, sensor và action status.

## Các bước trong video rút gọn

{times}

Mapping khảo sát tự động theo 17 waypoint, quãng đường {results['mapping']['distance_m']:.2f} m,
không bumper. Đây là tuyến có sẵn cho phòng vuông, không phải thao tác teleop bằng bàn phím.
SLAM dùng LiDAR và odometry; pose Gazebo chỉ giúp điều khiển tuyến khảo sát.
Sau khi lưu, map chỉ cắt vùng ngoài bốn tường đã đo, không vẽ/sửa occupancy bên trong;
map trước khi cắt được giữ trong `maps/map_raw.*`.
Dock được bỏ sau mỗi lần undock; cùng tường và ba vật cản được giữ khi khởi động lại.

SLAM đã dừng trước khi khởi động lại và load map. AMCL được đặt initial pose
gần vị trí đã biết sau undock; sau tuyến kiểm tra, sai số so với Gazebo là
{100*local['position_error_m']:.1f} cm và {local['yaw_error_deg']:.1f}°.
Các chuyển động kiểm tra localization kết thúc trước khi gửi goal Nav2.

| Goal | Đích thực tế trong map (m) | Action status | Sai số cuối |
| --- | --- | --- | --- |
{table}

Ba goal được gửi bằng {results['navigation_goal_source']}, lần lượt chờ SUCCEEDED
rồi mới gửi goal tiếp theo. Không phát lệnh vận tốc thủ công sau goal.
Hazard có {navigation['hazard_messages']} bản tin, {navigation['bump_messages']} bản tin bumper.
Inflation, hạt AMCL, kế hoạch xanh lá và đường đã đi màu cam được hiển thị.
Sai số cuối goal nằm trong tolerance của Nav2, không phải sai số localization.

Bản rút gọn tăng tốc mapping 4 lần, khởi động lại 4 lần, tuyến localization
2 lần, có nhãn tốc độ; toàn bộ navigation giữ tốc độ thực.
Bản đầy đủ có chú thích nhưng không chủ động tăng tốc. Không có âm thanh.
{results.get('editing_note','')}
''',encoding='utf-8')
    print('Full seconds:',duration,'Short seconds:',short_info['format']['duration'])

if __name__=='__main__':main()
