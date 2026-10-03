import os
from pathlib import Path
import sys


if __name__ == "__main__":
    print("Demo 2: Zone B is free. Request: Put the red cube in Zone B.", flush=True)
    os.execv(sys.executable, [sys.executable, str(Path(__file__).with_name("run_demo.py")),
                             *sys.argv[1:], "--scenario", "2"])
