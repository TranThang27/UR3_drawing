#!/usr/bin/env python3
"""Place a goal with RViz's actual mouse tool in the report view.

Pixel coordinates are explicit: this helper does not send ROS goals or robot
velocity commands. Verify the camera center/scale against a captured RViz view
before using it; moving panels or the camera changes these coordinates.
"""
import argparse
import ctypes as C
import json
import math
from pathlib import Path
import subprocess
import shutil
import time

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('x',type=float);parser.add_argument('y',type=float)
    parser.add_argument('yaw',type=float)
    parser.add_argument('--window',type=lambda s:int(s,0),required=True)
    parser.add_argument('--center',type=float,nargs=2,default=[960,588])
    parser.add_argument('--scale',type=float,default=125)
    parser.add_argument('--log',type=Path)
    args=parser.parse_args()
    locked=subprocess.run(['gdbus','call','--session','--dest','org.gnome.ScreenSaver',
        '--object-path','/org/gnome/ScreenSaver','--method','org.gnome.ScreenSaver.GetActive'],
        capture_output=True,text=True)
    if locked.returncode==0 and 'true' in locked.stdout:
        parser.error('Desktop is locked; unlock it before using the RViz goal tool.')
    root=Path(__file__).resolve().parents[1]
    subprocess.run([shutil.which('wmctrl') or str(root/'.runtime/root/usr/bin/wmctrl'),'-ia',hex(args.window)],check=True)
    time.sleep(.7)
    x=C.CDLL('libX11.so.6');t=C.CDLL('libXtst.so.6')
    x.XOpenDisplay.restype=C.c_void_p;display=x.XOpenDisplay(None)
    x.XDefaultRootWindow.argtypes=[C.c_void_p];x.XDefaultRootWindow.restype=C.c_ulong
    x.XTranslateCoordinates.argtypes=[C.c_void_p,C.c_ulong,C.c_ulong,C.c_int,C.c_int,
                                     C.POINTER(C.c_int),C.POINTER(C.c_int),C.POINTER(C.c_ulong)]
    ox,oy,child=C.c_int(),C.c_int(),C.c_ulong()
    x.XTranslateCoordinates(display,args.window,x.XDefaultRootWindow(display),0,0,
                            C.byref(ox),C.byref(oy),C.byref(child))
    t.XTestFakeMotionEvent.argtypes=[C.c_void_p,C.c_int,C.c_int,C.c_int,C.c_ulong]
    t.XTestFakeButtonEvent.argtypes=[C.c_void_p,C.c_uint,C.c_int,C.c_ulong]
    x.XFlush.argtypes=[C.c_void_p]
    def move(px,py):
        t.XTestFakeMotionEvent(display,-1,int(px)+ox.value,int(py)+oy.value,0);x.XFlush(display)
    def button(down):
        t.XTestFakeButtonEvent(display,1,int(down),0);x.XFlush(display)
    # Nav2 Goal has the 'g' shortcut; focus explicitly before sending it.
    x.XSetInputFocus.argtypes=[C.c_void_p,C.c_ulong,C.c_int,C.c_ulong]
    x.XStringToKeysym.argtypes=[C.c_char_p];x.XStringToKeysym.restype=C.c_ulong
    x.XKeysymToKeycode.argtypes=[C.c_void_p,C.c_ulong];x.XKeysymToKeycode.restype=C.c_uint
    t.XTestFakeKeyEvent.argtypes=[C.c_void_p,C.c_uint,C.c_int,C.c_ulong]
    x.XSetInputFocus(display,args.window,2,0)
    move(*args.center)
    key=x.XKeysymToKeycode(display,x.XStringToKeysym(b'g'))
    t.XTestFakeKeyEvent(display,key,1,0);t.XTestFakeKeyEvent(display,key,0,0)
    x.XFlush(display);time.sleep(.4)
    # A newly opened RViz panel may retain keyboard focus. Select the toolbar
    # tool explicitly as well, using its stable position in these configurations.
    move(557,44);button(True);time.sleep(.08);button(False);time.sleep(.3)
    px=args.center[0]+args.scale*args.x;py=args.center[1]-args.scale*args.y
    move(px,py);button(True);time.sleep(.15)
    move(px+45*math.cos(args.yaw),py-45*math.sin(args.yaw));time.sleep(.2);button(False)
    entry={'requested_map_xy_yaw':[args.x,args.y,args.yaw], 'window_pixel':[px,py],
           'window_origin':[ox.value,oy.value], 'method':'RViz Nav2 Goal tool mouse click and drag',
           'wall_time_unix':time.time()}
    path=args.log or root/'reports/part4/rviz_goal_clicks.json'
    path.parent.mkdir(parents=True,exist_ok=True)
    data=json.loads(path.read_text()) if path.exists() else []
    data.append(entry);path.write_text(json.dumps(data,indent=2)+'\n');print(entry)

if __name__=='__main__':main()
