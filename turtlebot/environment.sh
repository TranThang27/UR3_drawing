#!/usr/bin/env bash
# Sourced by run.sh; ROS system dependencies plus the private runtime overlay.
TB4_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${TB4_ENVIRONMENT_READY:-}" == "$TB4_ROOT" ]]; then
  return 0
fi
source /opt/ros/humble/setup.bash
TB4_RUNTIME="$TB4_ROOT/.runtime/root"
if [[ -f "$TB4_ROOT/.runtime/manifest.json" ]]; then
  source "$TB4_RUNTIME/opt/ros/humble/local_setup.bash"
  export LD_LIBRARY_PATH="$TB4_RUNTIME/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
  export QML2_IMPORT_PATH="$TB4_RUNTIME/usr/lib/x86_64-linux-gnu/qt5/qml:${QML2_IMPORT_PATH:-}"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$TB4_ROOT/config/cyclonedds_local.xml}"
# Prefer NVIDIA when its driver is usable; the Ogre2 sensor backend also runs on Intel.
if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1; then
  export __NV_PRIME_RENDER_OFFLOAD=1
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
fi
export TB4_ENVIRONMENT_READY="$TB4_ROOT"
