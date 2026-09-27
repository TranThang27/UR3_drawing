# UR3 LLM Robot

This branch contains the MoveIt 2 pick-and-place simulation, Robotiq gripper,
and the Gemini natural-language command node. The drawing application remains
on the `main` branch.

```bash
git clone --branch LLM_robot --recurse-submodules \
  git@github.com:TranThang27/UR3_drawing.git
cd UR3_drawing
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Add the Gemini key without committing it:

```bash
export GEMINI_API_KEY="YOUR_KEY"
```

Run the complete demo:

```bash
cd llm_HRI
python3 run_demo.py
```

The demo moves Red → A, Blue → B and Yellow → C, then returns home.
