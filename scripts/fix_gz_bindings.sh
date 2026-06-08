#!/usr/bin/env bash
set -euo pipefail

LOG="fix_gz_bindings.log"
echo "Starting fix_gz_bindings - logs -> $LOG"

# Source ROS setup if present
if [ -f /opt/ros/jazzy/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
else
  echo "Warning: /opt/ros/jazzy/setup.bash not found." | tee -a "$LOG"
fi

echo "Checking gz sim version" | tee "$LOG"
gz sim --version 2>&1 | tee -a "$LOG" || true

echo "Updating apt and installing candidate packages (may prompt for sudo)..." | tee -a "$LOG"
sudo apt update 2>&1 | tee -a "$LOG"

PKGS=(python3-gz python3-gz-sim6 python3-gz-math6 ros-jazzy-ros-gz ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge)
echo "Packages to install: ${PKGS[*]}" | tee -a "$LOG"
sudo apt install -y "${PKGS[@]}" 2>&1 | tee -a "$LOG" || true

echo "Testing Python gz imports" | tee -a "$LOG"
python3 - <<'PY' 2>>"$LOG"
try:
    import gz.transport13, gz.msgs10
    print('gz bindings OK')
except Exception as e:
    import sys
    print('IMPORT_FAILED', repr(e), file=sys.stderr)
    sys.exit(2)
PY

if [ $? -ne 0 ]; then
  echo "Python import for gz failed. See $LOG for details." | tee -a "$LOG"
  exit 1
fi

echo "Python imports OK." | tee -a "$LOG"

echo "Launching headless simulation for 15s to verify behavior..." | tee -a "$LOG"
ros2 launch hiking_lora_sim hiking_lora_sim.launch.py gz_args:="-r -v 3 --headless" > ros_launch_test.log 2>&1 &
LAUNCH_PID=$!
echo "Launched ros2 launch (pid=$LAUNCH_PID)" | tee -a "$LOG"
sleep 15

echo "----- LAUNCH LOG (last 200 lines) -----" >> "$LOG"
tail -n 200 ros_launch_test.log >> "$LOG" || true

echo "Searching for key indicators in launch log:" | tee -a "$LOG"
grep -E "Hiker GPS simulator started|Spawned Gazebo model|Gazebo Python transport is unavailable|process has died" ros_launch_test.log || true

echo "Stopping launch (pid=$LAUNCH_PID)" | tee -a "$LOG"
kill "$LAUNCH_PID" 2>/dev/null || true
sleep 2
pkill -f 'gz sim' 2>/dev/null || true

echo "Done. Review $LOG and ros_launch_test.log for details." | tee -a "$LOG"

exit 0
