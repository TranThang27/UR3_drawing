#!/usr/bin/env python3
"""Record actual Gazebo/RViz X11 windows into a labelled H.264 video."""
import argparse
import ctypes as C
import json
from pathlib import Path
import subprocess
import time
from PIL import Image, ImageDraw, ImageFont, ImageOps
from capture_x11_window import WindowCapture

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--state',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--fps',type=int,default=10)
    args=parser.parse_args()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    # A window may disappear during a simulator restart; report it as loading.
    x=C.CDLL('libX11.so.6')
    error_type=C.CFUNCTYPE(C.c_int,C.c_void_p,C.c_void_p)
    error_handler=error_type(lambda display,error:0)
    x.XSetErrorHandler.argtypes=[error_type];x.XSetErrorHandler(error_handler)
    encoder=subprocess.Popen(['ffmpeg','-y','-hide_banner','-loglevel','warning',
        '-f','rawvideo','-pix_fmt','rgb24','-s','1920x1080','-r',str(args.fps),
        '-i','pipe:0','-an','-c:v','libx264','-preset','veryfast','-crf','20',
        '-pix_fmt','yuv420p','-movflags','+faststart',str(args.output)],stdin=subprocess.PIPE)
    fonts=[ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',s) for s in (28,22,18)]
    captures={};start=time.monotonic();frames=0;missed=0
    try:
        while True:
            state=json.loads(args.state.read_text())
            if state.get('stop'):break
            canvas=Image.new('RGB',(1920,1080),'#18212c');draw=ImageDraw.Draw(canvas)
            draw.text((22,10),'TurtleBot 4  |  SLAM Toolbox → AMCL → Nav2',font=fonts[0],fill='white')
            draw.text((22,48),state.get('phase','Đang chuẩn bị'),font=fonts[1],fill='#8ae5ff')
            active=set()
            for key,box,label in [('gazebo',(0,100,600,1000),'Gazebo — vị trí thực tế'),
                                   ('rviz',(600,100,1920,1000),'RViz2 — map / sensor / navigation')]:
                draw.text((box[0]+15,80),label,font=fonts[2],fill='#dde5ef')
                window=state.get(key)
                if not window:
                    draw.text((box[0]+30,450),'Đang khởi động / chuyển bước…',font=fonts[1],fill='white');continue
                active.add(window)
                try:
                    if window not in captures:captures[window]=WindowCapture(window)
                    frame=captures[window].grab()
                    frame=ImageOps.contain(frame,(box[2]-box[0],box[3]-box[1]),Image.LANCZOS)
                    canvas.paste(frame,(box[0]+(box[2]-box[0]-frame.width)//2,
                                        box[1]+(box[3]-box[1]-frame.height)//2))
                except RuntimeError:
                    missed+=1
            for window in list(captures):
                if window not in active:captures.pop(window).close()
            draw.text((22,1010),state.get('detail',''),font=fonts[1],fill='white')
            draw.text((22,1044),state.get('legend','Ảnh cửa sổ thực tế • mô phỏng simple_square • không âm thanh'),font=fonts[2],fill='#b5c4d6')
            draw.text((1760,1044),f'{frames/args.fps:.1f} s',font=fonts[2],fill='#b5c4d6')
            encoder.stdin.write(canvas.tobytes());frames+=1
            delay=start+frames/args.fps-time.monotonic()
            if delay>0:time.sleep(delay)
    finally:
        for capture in captures.values():capture.close()
        encoder.stdin.close();encoder.wait()
        stats={'frames':frames,'fps':args.fps,'video_duration_s':frames/args.fps,
               'wall_duration_s':time.monotonic()-start,'missing_window_frames':missed,
               'encoder_exit_code':encoder.returncode,'source':'Actual X11 Gazebo and RViz windows'}
        args.output.with_suffix('.recording.json').write_text(json.dumps(stats,indent=2)+'\n')
    if encoder.returncode:raise RuntimeError('Video encoder failed')

if __name__=='__main__':main()
