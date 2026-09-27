# Run demo

```bash
cd ~/workspaces/llm_HRI
source /opt/ros/humble/setup.bash
source ~/workspaces/ur3_gz/install/setup.bash
python3 run_demo.py
```

Red → A, Blue → B, Yellow → C, then home.
Close previous launches first. Completion, failure or Ctrl+C cleans up this demo's simulation.

The default motion speed is 2x. Override it from 0.5x to 3x:

```bash
HRI_SPEED_SCALE=3 python3 run_demo.py
```

See [run instructions](../README.md) for manual mode and English commands.
