# UR3 RGB-D Camera Robot — LLM_HRI2

This branch adds `LLM_HRI_2`: five colored cubes, three zones, an RGB-D camera,
and automatic clearance of occupied destinations using camera-selected temporary
positions and MoveIt 2. The previous `llm_HRI` version is retained separately.

```bash
git clone --branch LLM_HRI2 --recurse-submodules \
  git@github.com:TranThang27/UR3_drawing.git
cd UR3_drawing
source /opt/ros/humble/setup.bash
colcon build --packages-select robotiq_description ur_simulation_gz --symlink-install
source install/setup.bash
```

Install camera dependencies if needed:

```bash
sudo apt install ros-humble-cv-bridge ros-humble-message-filters python3-opencv python3-numpy
```

Run one demo at a time (no Gemini key required):

```bash
cd LLM_HRI_2
python3 demo1.py
```

Demo 1: blue occupies B; move blue to a camera-verified free tabletop patch,
then place red into B.

```bash
python3 demo2.py
```

Demo 2: B is empty; pick red and place it directly into B.
Both scenarios use all five cubes and stop their simulation on completion.
Add `--headless` to run without GUI windows.

For natural-language commands, set `GEMINI_API_KEY` or `GEMINI_KEY_FILE`, then:

```bash
python3 demo1.py --command "Put the red cube in Zone B."
```

See [LLM_HRI_2 instructions](LLM_HRI_2/README.md) for camera topics and tests.
