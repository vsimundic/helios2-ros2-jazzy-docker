#!/usr/bin/env bash
set -e

ARENA="${ARENA_PATH:-/opt/ArenaSDK_Linux_x64}"

# GenTL producers (cti)
export GENICAM_GENTL64_PATH="${GENICAM_GENTL64_PATH:-$ARENA/lib64}"

# Force X11
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

# 1) Prefer bundled Qt (AVMP requires Qt_6.7, Ubuntu noble Qt is older)
QT_CORE="$(find -L "$ARENA" -type f -name 'libQt6Core.so.6' -print -quit 2>/dev/null || true)"
if [ -n "$QT_CORE" ]; then
  QT_LIBDIR="$(dirname "$QT_CORE")"
  export LD_LIBRARY_PATH="$QT_LIBDIR:${LD_LIBRARY_PATH}"
fi

# 2) Ensure Qt can find platform plugins (xcb)
QXCB="$(find -L "$ARENA" -type f -name 'libqxcb.so' -print -quit 2>/dev/null || true)"
if [ -n "$QXCB" ]; then
  export QT_PLUGIN_PATH="$(dirname "$(dirname "$QXCB")")"
fi

# 3) AVMP bundled libs (libarena, metavision, ffmpeg, opencv, etc.)
export LD_LIBRARY_PATH="$ARENA/OutputDirectory/Linux/x64Release:$ARENA/lib64:$ARENA/Metavision/lib:$ARENA/ffmpeg:$ARENA/OpenCV/lib:${LD_LIBRARY_PATH}"

# Chromium/CEF stability in containers
export QTWEBENGINE_DISABLE_SANDBOX=1
export QTWEBENGINE_CHROMIUM_FLAGS="--no-sandbox --disable-dev-shm-usage --disable-features=Vulkan"

# Chromium/CEF flags (QCefView/CEF reads CLI switches)
DEFAULT_ARGS=(
  --no-sandbox
  --disable-gpu
  --disable-gpu-compositing
  --disable-gpu-sandbox
  --disable-dev-shm-usage
  --disable-features=Vulkan
  --in-process-gpu
  --enable-logging=stderr
  --v=1
)

cd "$ARENA/OutputDirectory/Linux/x64Release"
exec ./ArenaViewMP "${DEFAULT_ARGS[@]}" "$@"
