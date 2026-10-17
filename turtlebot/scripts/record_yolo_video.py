#!/usr/bin/env python3
"""Record Gazebo, RViz2 and the real YOLO camera window simultaneously."""
import argparse
import ctypes as C
import json
from pathlib import Path
import subprocess
import time
from PIL import Image, ImageDraw, ImageFont, ImageOps
from capture_x11_window import WindowCapture


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--state', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--fps', type=int, default=5)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    error_type = C.CFUNCTYPE(C.c_int, C.c_void_p, C.c_void_p)
    handler = error_type(lambda display, error: 0)
    x = C.CDLL('libX11.so.6')
    x.XSetErrorHandler.argtypes = [error_type]
    x.XSetErrorHandler(handler)
    encoder = subprocess.Popen(['ffmpeg', '-y', '-hide_banner', '-loglevel', 'warning',
        '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', '1920x1080', '-r', str(args.fps),
        '-i', 'pipe:0', '-an', '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '20',
        '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(args.output)], stdin=subprocess.PIPE)
    fonts = [ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', s)
             for s in (27, 21, 18)]
    captures = {}; frames = missing = 0; started = time.monotonic(); started_wall = time.time()
    try:
        while True:
            state = json.loads(args.state.read_text())
            if state.get('stop'):
                break
            canvas = Image.new('RGB', (1920, 1080), '#18212c')
            draw = ImageDraw.Draw(canvas)
            draw.text((20, 8), 'TurtleBot 4 | Load map → AMCL → 3 goals bằng code + YOLO11',
                      font=fonts[0], fill='white')
            draw.text((20, 44), state.get('phase', ''), font=fonts[1], fill='#8ae5ff')
            for key, box, label in [
                ('gazebo', (0, 104, 880, 540), 'Gazebo — robot và vật thể trong môi trường'),
                ('yolo', (0, 572, 880, 1005), 'Camera RGB của robot — YOLO bounding box / class / confidence'),
                ('rviz', (880, 104, 1920, 1005), 'RViz2 — saved map / AMCL / inflation / đường đi')]:
                draw.text((box[0] + 12, box[1] - 25), label, font=fonts[2], fill='#dde5ef')
                window = state.get(key)
                if window:
                    try:
                        if window not in captures:
                            captures[window] = WindowCapture(window)
                        image = captures[window].grab()
                        image = ImageOps.contain(image, (box[2]-box[0], box[3]-box[1]))
                        canvas.paste(image, (box[0]+(box[2]-box[0]-image.width)//2,
                                            box[1]+(box[3]-box[1]-image.height)//2))
                    except RuntimeError:
                        missing += 1
                else:
                    draw.text((box[0]+20, box[1]+60), 'Đang khởi chạy…', font=fonts[1], fill='white')
            draw.text((20, 1017), state.get('detail', ''), font=fonts[1], fill='white')
            draw.text((20, 1050), 'Ảnh cửa sổ thực tế • map từ tuần trước • goal qua /navigate_to_pose • không teleop',
                      font=fonts[2], fill='#b5c4d6')
            draw.text((1780, 1050), f'{frames/args.fps:.1f} s', font=fonts[2], fill='#b5c4d6')
            encoder.stdin.write(canvas.tobytes()); frames += 1
            delay = started + frames/args.fps - time.monotonic()
            if delay > 0:
                time.sleep(delay)
    finally:
        for capture in captures.values():
            capture.close()
        encoder.stdin.close(); encoder.wait()
        args.output.with_suffix('.recording.json').write_text(json.dumps({
            'frames': frames, 'fps': args.fps, 'duration_s': frames/args.fps,
            'wall_duration_s': time.monotonic()-started, 'started_wall_time': started_wall,
            'ended_wall_time': time.time(), 'missing_window_frames': missing,
            'encoder_exit_code': encoder.returncode,
            'source': 'Actual X11 Gazebo, RViz2 and YOLO OpenCV windows'}, indent=2)+'\n')
    if encoder.returncode:
        raise RuntimeError('Video encoder failed')


if __name__ == '__main__':
    main()
