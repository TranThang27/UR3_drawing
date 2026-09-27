# UR3 tabletop pick-and-place world

This package includes a simple Gazebo Fortress environment containing:

- one UR3 or UR3e with the standard Robotiq 2F-85 description and UR adapter;
- `red_cube`, `yellow_cube`, and `blue_cube` (kinematic 50 mm cubes);
- visual-only target areas named `zone_a`, `zone_b`, and `zone_c`.

Build and launch from the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch ur_simulation_gz pick_place.launch.py
```

The default robot is UR3e. To use UR3 or run without the Gazebo GUI:

```bash
ros2 launch ur_simulation_gz pick_place.launch.py ur_type:=ur3
ros2 launch ur_simulation_gz pick_place.launch.py gazebo_gui:=false
```

The table surface and robot base are at `z=0.75 m`. Zone A/B/C are green,
orange, and purple respectively. The target zones have no collision geometry,
and cube collision checking is handled by the MoveIt planning scene. Gazebo
receives cube poses through the `set_pose` bridge while a cube is carried.

To open Gazebo and MoveIt/RViz together, including all table, cube, and zone
geometry in the MoveIt planning scene:

```bash
ros2 launch ur_simulation_gz pick_place_moveit.launch.py
```

To run MoveIt without its RViz window, pass `moveit_rviz:=false`.
