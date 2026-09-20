# UR Robot Drawing

```bash
cd /home/acer/ur_robot_drawing-main/UR3_drawing
source /opt/ros/humble/setup.bash
colcon build --packages-select ur_simulation_gz ur_drawing --symlink-install
source install/setup.bash
ros2 launch ur_drawing draw_p.launch.py
```

