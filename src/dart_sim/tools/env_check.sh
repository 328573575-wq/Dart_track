#!/usr/bin/env bash
# Check that this machine can build and run the dart_sim simulation.
# Usage:  bash tools/env_check.sh
set +e

echo "== system =="
uname -a
echo "distro : $(. /etc/os-release 2>/dev/null && echo "$PRETTY_NAME")"
echo "python : $(python3 --version 2>&1)"

echo
echo "== ROS 2 =="
if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    echo "ROS_DISTRO = ${ROS_DISTRO:-?}"
    ros2 --help >/dev/null 2>&1 && echo "ros2       : ok" || echo "ros2       : MISSING"
else
    echo "no /opt/ros/humble/setup.bash"
fi

echo
echo "== python packages =="
python3 - <<'PY'
for name in ("numpy", "yaml", "cv2", "cv_bridge"):
    try:
        module = __import__(name)
        print(f"{name:10s}: {getattr(module, '__version__', 'ok')}")
    except Exception as exc:  # noqa: BLE001
        print(f"{name:10s}: MISSING ({exc})")
PY

echo
echo "== build tools =="
command -v colcon >/dev/null 2>&1 && echo "colcon    : $(command -v colcon)" || echo "colcon    : MISSING (sudo apt install python3-colcon-common-extensions)"
command -v ros2 >/dev/null 2>&1 && echo "ros2      : $(command -v ros2)" || echo "ros2      : MISSING"

echo
echo "== display =="
echo "DISPLAY=${DISPLAY:-<unset>}  WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-<unset>}"
echo "done"

