#!/usr/bin/env python3
"""Validate the completed recording and export a labelled shorter demo."""
import argparse
import json
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    args = parser.parse_args(); folder = args.folder.resolve()
    report = json.loads((folder / 'demo_results.json').read_text())
    if not report['passed']:
        raise RuntimeError('Demo did not pass; do not export a success video')
    video = folder / 'turtlebot4_nav_yolo.mp4'
    probe = json.loads(subprocess.check_output(['ffprobe','-v','error','-show_streams','-show_format','-of','json',str(video)],text=True))
    duration = float(probe['format']['duration'])
    stream = next(s for s in probe['streams'] if s['codec_type']=='video')
    if stream['width']!=1920 or stream['height']!=1080 or stream['codec_name']!='h264':
        raise RuntimeError('Unexpected recording format')
    chapters = json.loads((folder / 'chapters.json').read_text())
    mission_start = next(c['start_s'] for c in chapters if c['title'].startswith('2/3'))
    completion = next(c['start_s'] for c in chapters if c['title'].startswith('3/3'))
    if not 0 < mission_start < completion < duration:
        raise RuntimeError('Chapter timestamps do not fit recording')
    stats = json.loads(video.with_suffix('.recording.json').read_text())
    started_wall = stats.get('started_wall_time') or float(subprocess.check_output(['stat', '-c', '%W', str(video)], text=True))
    if started_wall <= 0:
        raise RuntimeError('Recording start time is unavailable')
    start_wall = started_wall + mission_start
    end_wall = started_wall + completion
    during = [json.loads(line) for line in (folder / 'yolo/detections.jsonl').read_text().splitlines()]
    during = [frame for frame in during if start_wall <= frame['wall_time'] <= end_wall]
    detected = sum(bool(frame['objects']) for frame in during)
    if len(during) < 20 or detected < 5:
        raise RuntimeError('Insufficient real YOLO frames/detections during navigation')
    report['yolo_during_navigation'] = {'frames': len(during), 'frames_with_detections': detected}
    (folder / 'demo_results.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    filters = [
        f'[0:v]trim=start=0:end={mission_start},setpts=PTS-STARTPTS[a]',
        f"[0:v]trim=start={mission_start}:end={completion},setpts=(PTS-STARTPTS)/3,"
        "drawtext=fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf:"
        "text='NAVIGATION TUA 3x':x=1550:y=50:fontsize=21:fontcolor=yellow:box=1:boxcolor=black@0.7[b]",
        f'[0:v]trim=start={completion}:end={duration},setpts=PTS-STARTPTS[c]',
        '[a][b][c]concat=n=3:v=1:a=0,fps=15[out]']
    short = folder / 'turtlebot4_nav_yolo_short.mp4'
    subprocess.run(['ffmpeg','-y','-hide_banner','-loglevel','warning','-i',str(video),
        '-filter_complex',';'.join(filters),'-map','[out]','-an','-c:v','libx264',
        '-preset','veryfast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(short)],check=True)
    # Decode the entire videos to verify there are no damaged frames.
    for path in [video, short]:
        subprocess.run(['ffmpeg','-v','error','-i',str(path),'-f','null','-'],check=True)
    short_probe = json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries','format=duration','-of','json',str(short)],text=True))
    results = {'full_duration_s':duration,'short_duration_s':float(short_probe['format']['duration']),
               'width':1920,'height':1080,'source':'Real Gazebo/RViz2/YOLO windows',
               'short_note':'Only the navigation segment is sped up 3x with an on-screen label.'}
    (folder / 'video_info.json').write_text(json.dumps(results,indent=2)+'\n')
    lines = ['# TurtleBot 4 — Nav2 bằng code + YOLO', '',
             '- Full: `turtlebot4_nav_yolo.mp4` (thời gian thực).',
             '- Short: `turtlebot4_nav_yolo_short.mp4` (navigation tua 3x, có ghi nhãn).',
             '- Kết quả: `demo_results.json`; map cũ được giữ nguyên.', '',
             '| Goal | x | y | yaw (rad) | Status | Sai số vị trí (m) |',
             '|---|---:|---:|---:|---|---:|']
    for i, entry in enumerate(report['navigation']['goals'],1):
        x,y,yaw=entry['goal']
        lines.append(f'| {i} | {x} | {y} | {yaw} | SUCCEEDED | {entry["position_error_m"]:.3f} |')
    lines.extend(['',f'Bumper: {report["navigation"]["observations"]["bump_messages"]}.',
                  f'YOLO: {report["yolo"]["frames_processed"]} frames, classes {report["yolo"]["class_counts"]}.', '',
                  '## Mốc trong video đầy đủ', ''])
    for chapter in chapters:
        seconds=int(chapter['start_s']);lines.append(f'- {seconds//60:02}:{seconds%60:02} — {chapter["title"]}')
    (folder/'README.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(results),flush=True)


if __name__ == '__main__': main()
