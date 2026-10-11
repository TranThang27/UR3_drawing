#!/usr/bin/env python3
"""Remove the default simulator dock after the robot has undocked."""
import argparse
import os
from pathlib import Path
import subprocess

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world',default='simple_square')
    parser.add_argument('--partition',default=os.environ.get('IGN_PARTITION'))
    args=parser.parse_args()
    if not args.partition:
        candidates=set()
        for path in Path('/proc').glob('[0-9]*/cmdline'):
            try:
                cmd=path.read_bytes().replace(b'\x00',b' ')
                if b'ign gazebo server' not in cmd:continue
                for entry in path.with_name('environ').read_bytes().split(b'\x00'):
                    if entry.startswith(b'IGN_PARTITION='):
                        candidates.add(entry.split(b'=',1)[1].decode())
            except (PermissionError,ProcessLookupError,FileNotFoundError):pass
        if len(candidates)!=1:
            parser.error('Specify --partition for the running simulator.')
        args.partition=candidates.pop()
    result=subprocess.run(['ign','service','-s',f'/world/{args.world}/remove',
        '--reqtype','ignition.msgs.Entity','--reptype','ignition.msgs.Boolean',
        '--timeout','5000','--req','name: "standard_dock", type: MODEL'],
        env={**os.environ,'IGN_PARTITION':args.partition},capture_output=True,text=True,timeout=10)
    if result.returncode or 'data: true' not in result.stdout:
        raise RuntimeError('Gazebo did not confirm dock removal: '+result.stdout+result.stderr)
    print('Đã xóa standard_dock khỏi Gazebo sau undock.')

if __name__=='__main__':main()
