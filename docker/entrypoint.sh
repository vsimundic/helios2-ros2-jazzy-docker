#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash

export ARENA_PATH="${ARENA_PATH:-/opt/ArenaSDK_Linux_x64}"
export ARENA_ROOT="${ARENA_ROOT:-/opt/avmp}"
export WS="${WS:-/ws}"

# GenTL producer CTI files are here in your bundle
if [ -d "${ARENA_PATH}/lib64" ]; then
  export GENICAM_GENTL64_PATH="${ARENA_PATH}/lib64"
fi

# Optional: build ROS workspace on first run (only if /ws/src exists and is non-empty)
if [ -d "$WS/src" ] && [ -n "$(ls -A "$WS/src" 2>/dev/null)" ]; then
  cd "$WS"
  if [ ! -f "$WS/install/local_setup.bash" ]; then
    rosdep update || true
    rosdep install --from-paths src --ignore-src -r -y || true
    colcon build --symlink-install
  fi
  source "$WS/install/local_setup.bash" || true
fi

exec "$@"
