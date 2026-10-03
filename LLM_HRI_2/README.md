# LLM_HRI_2: camera-guided pick and place

## Two demonstration scenarios

Both scenarios contain red, yellow, blue, green and purple cubes and request
`Put the red cube in Zone B.` The camera determines the execution steps.

```bash
cd ~/workspaces/LLM_HRI_2
python3 demo1.py
```

Demo 1: B initially contains blue. The robot observes B, relocates blue to a
camera-selected empty tabletop position, then picks red and places it in B.

```bash
python3 demo2.py
```

Demo 2: blue initially rests elsewhere on the table and B is empty. The robot
observes B and transfers red directly, without moving another cube.
Purple occupies C in both scenarios; yellow and green remain on the table.
Spawn layouts configure Gazebo only; the planner reads positions from RGB-D.

Use `--headless` to disable GUI windows or `--observe` to inspect without motion.
Run one demo at a time. Each demo automatically stops its simulation when finished.
The default request is deterministic and requires no Gemini API call. To use Gemini:

```bash
export GEMINI_KEY_FILE=~/workspaces/llm_HRI/llm/api.txt
python3 demo1.py --command "Put the red cube in Zone B."
```

ROS 2 Humble, Gazebo Fortress, MoveIt 2 and the existing `ur3_gz` workspace are required.
This folder is independent of `llm_HRI`; it reuses the installed UR3e/Robotiq description
and controllers but supplies its own world, camera, perception and planning-scene loader.

Install camera dependencies if missing:

```bash
sudo apt install ros-humble-cv-bridge ros-humble-message-filters ros-humble-ros-gz-bridge python3-opencv python3-numpy
```

Run the camera-driven demo (no Gemini call):

```bash
cd ~/workspaces/LLM_HRI_2
python3 run_demo.py
```

The initial scene has five 50-mm cubes. Zone B contains blue, Zone C contains
purple, and red/yellow/green are elsewhere on the table. The demo requests red → B.
The executor observes B, finds an empty tabletop patch, checks approach/grasp IK,
moves blue to that patch, re-observes B, then transfers red and verifies the result.
All generated skill steps and statuses are printed. Completion, failure or Ctrl+C
cleans up the processes started by the demo. `--headless` disables GUI windows.

Inspect the camera state without motion:

```bash
python3 run_demo.py --observe --headless
```

English natural-language command:

```bash
export GEMINI_KEY_FILE=~/workspaces/llm_HRI/llm/api.txt
python3 run_demo.py --command "Put the red cube in Zone B."
```

Alternatively set `GEMINI_API_KEY`. No key is copied into this folder.
The LLM emits logical pick/place/home steps; the executor expands transfers from
current camera evidence. It never asks the LLM to invent object coordinates.

For a persistent simulation, terminal 1:

```bash
source /opt/ros/humble/setup.bash
source ~/workspaces/ur3_gz/install/setup.bash
ros2 launch ~/workspaces/LLM_HRI_2/launch/camera_moveit.launch.py
```

Terminal 2 (source the same setup files):

```bash
cd ~/workspaces/LLM_HRI_2
python3 main.py --observe
python3 main.py --demo
```

`python3 main.py` runs the language node: input `/hri2/command` and output
`/hri2/result`, both `std_msgs/msg/String`. Do not run simultaneous command clients.
The sensor publishes `/hri_camera/image`, `/hri_camera/depth_image`,
`/hri_camera/camera_info`; annotated RGB appears in RViz. Perception publishes
JSON `/hri2/world_state`: `objects`, `zones` (`FREE`, `OCCUPIED`, `UNKNOWN`),
`free_positions`, frame stamp, missing detections and observation time.

The camera uses calibrated intrinsics and a fixed extrinsic transform to the robot
planning frame. Only the table, camera calibration and object dimensions are known
in advance. Cube positions and zone centres come from synchronized RGB-D frames.
Cube/zone separation uses color and measured height; no Gazebo model-pose topic,
SDF object-position lookup, object teleportation or physical attachment is used.
Gazebo spawn coordinates define the initial scene only.

Free-position candidates require visible table depth across the whole footprint,
clearance from every observed cube and zone, and a reachable workspace. Missing,
ambiguous or stale observations stop execution; absence of a detection is not
treated as evidence that a zone is free. The robot returns to its observation
pose between transfers so its arm does not hide the other cubes. Grasp monitoring
uses visible cube surfaces and gripper feedback; release is checked again by camera.

This detector is for the five uniquely colored cubes in this calibrated simulated
scene, not arbitrary household objects. Moving the camera requires updating its
extrinsics in `vision.py`. Occlusion, stacked cubes, duplicate colors or cubes
outside the monitored workspace may produce `PERCEPTION_FAILED`. No empty reachable
patch produces a failure before picking the blocker. Planning remains collision-aware.

Tests (no simulation):

```bash
python3 -m unittest test_camera_logic -v
```

Camera bridge reference: https://gazebosim.org/docs/fortress/ros2_integration/
