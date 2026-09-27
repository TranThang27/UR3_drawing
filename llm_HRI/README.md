# LLM robot commands

Build the repository first:

```bash
cd ~/workspaces/UR3_drawing
git submodule update --init --recursive
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Set `GEMINI_API_KEY` or create the ignored file `llm_HRI/llm/api.txt`.

Run Red → A, Blue → B, Yellow → C:

```bash
cd ~/workspaces/UR3_drawing/llm_HRI
python3 run_demo.py
```

Run one English command while the simulation is available:

```bash
python3 main.py --command "Move the red cube to zone A."
```

Preview the generated plan without robot motion:

```bash
python3 main.py --command "Move the red cube to zone A." --dry-run
```

Start the ROS 2 command node:

```bash
python3 main.py
```

Send a command from another sourced terminal:

```bash
ros2 topic pub --once /hri/command std_msgs/msg/String \
  "data: 'Move the red cube to zone A.'"
```

Read results:

```bash
ros2 topic echo /hri/result
```
