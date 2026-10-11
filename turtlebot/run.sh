#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'Cần ROS 2 Humble tại /opt/ros/humble.' >&2
  exit 1
fi
source "$ROOT/environment.sh"
set -u

require_pkg() {
  if ! ros2 pkg prefix "$1" >/dev/null 2>&1; then
    echo "Thiếu package $1. Xem README.md, bước cài dependencies." >&2
    exit 1
  fi
}

COMMAND="${1:-help}"
if (($#)); then shift; fi
case "$COMMAND" in
  setup)
    exec /usr/bin/python3 "$ROOT/scripts/setup_local.py"
    ;;
  sim)
    require_pkg turtlebot4_ignition_bringup
    exec ros2 launch "$ROOT/launch/simulation.launch.py" "$@"
    ;;
  demo)
    exec python3 "$ROOT/scripts/run_demo.py" "$@"
    ;;
  mapping)
    exec python3 "$ROOT/scripts/run_demo.py" --mapping "$@"
    ;;
  auto-map)
    exec python3 "$ROOT/scripts/run_demo.py" --mapping --auto-survey "$@"
    ;;
  navigation)
    exec python3 "$ROOT/scripts/run_demo.py" --navigation "$@"
    ;;
  video-demo)
    exec python3 "$ROOT/scripts/run_video_demo.py" "$@"
    ;;
  goals)
    exec python3 "$ROOT/scripts/check_navigation.py" "$@"
    ;;
  check|motion)
    if [[ "$COMMAND" == motion ]]; then
      exec python3 "$ROOT/scripts/check_simulation.py" --motion "$@"
    fi
    exec python3 "$ROOT/scripts/check_simulation.py" "$@"
    ;;
  hz)
    exec ros2 topic hz "$@"
    ;;
  teleop)
    require_pkg teleop_twist_keyboard
    exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel -p speed:=0.2 -p turn:=0.5 -p repeat_rate:=10.0 -p key_timeout:=0.8 "$@"
    ;;
  undock)
    require_pkg irobot_create_msgs
    exec python3 "$ROOT/scripts/undock_robot.py"
    ;;
  slam)
    require_pkg turtlebot4_navigation
    require_pkg slam_toolbox
    exec ros2 launch turtlebot4_navigation slam.launch.py use_sim_time:=true sync:=true "params:=$ROOT/config/slam.yaml" "$@"
    ;;
  save)
    require_pkg nav2_map_server
    exec python3 "$ROOT/scripts/save_map.py" "${1:-$ROOT/maps/map}"
    ;;
  nav|localize)
    require_pkg turtlebot4_navigation
    MAP_PATH="${1:-$ROOT/maps/map.yaml}"
    if [[ ! -f "$MAP_PATH" ]]; then
      echo "Không tìm thấy map: $MAP_PATH. Chạy SLAM và save trước." >&2
      exit 1
    fi
    if (($#)); then shift; fi
    MAP_PATH="$(realpath -- "$MAP_PATH")"
    NAVIGATION=true
    if [[ "$COMMAND" == localize ]]; then NAVIGATION=false; fi
    exec ros2 launch "$ROOT/launch/navigation.launch.py" "map:=$MAP_PATH" "navigation:=$NAVIGATION" "$@"
    ;;
  remove-dock)
    exec python3 "$ROOT/scripts/remove_dock.py" "$@"
    ;;
  nav2)
    require_pkg turtlebot4_navigation
    NAV2_PARAMS="$(ros2 pkg prefix turtlebot4_navigation)/share/turtlebot4_navigation/config/nav2.yaml"
    exec ros2 launch turtlebot4_navigation nav2.launch.py use_sim_time:=true "params_file:=$NAV2_PARAMS" "$@"
    ;;
  doctor)
    failed=0
    for pkg in turtlebot4_ignition_bringup irobot_create_nodes turtlebot4_navigation slam_toolbox nav2_bringup nav2_map_server rviz2 teleop_twist_keyboard; do
      if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
        echo "OK      $pkg"
      else
        echo "MISSING $pkg"
        failed=1
      fi
    done
    echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
    ign gazebo --versions || failed=1
    exit "$failed"
    ;;
  help|-h|--help)
    echo 'Usage: ./run.sh {setup|doctor|demo|mapping|auto-map|navigation|video-demo|sim|check|hz|undock|remove-dock|motion|teleop|slam|save|localize|nav2|nav|goals}'
    echo 'hz: /scan or /odom (Ctrl+C to finish)'
    echo 'navigation: [--map maps/map.yaml] [world:=simple_square]'
    echo 'sim: world:=simple_square model:=standard rviz:=true (also world:=depot)'
    echo 'check/motion: --timeout 15 --report reports/check.json'
    echo 'save: [map_prefix]; nav: [map.yaml]'
    ;;
  *) echo "Lệnh không hợp lệ: $COMMAND" >&2; exit 2 ;;
esac
