#!/usr/bin/env bash
set -e
cd /rai
source /opt/ros/jazzy/setup.bash
export SHELL=/bin/bash
source ./setup_shell.sh
set -a
source /run/secrets/deepseek.env
set +a
export OPENAI_API_KEY="$DEEPSEEK_API_KEY"
export ROS_DOMAIN_ID=73
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROBOWEAVE_ENABLE_FAULTS=1
export DISPLAY=:91
export QT_QPA_PLATFORM=xcb
mkdir -p artifacts/roboweave
# This display and /tmp directory belong exclusively to this container.
# A forced container stop can leave X11's lock/socket behind.
python - <<'PY'
from pathlib import Path
p=Path('/tmp/.X91-lock')
if p.exists():
    pid=p.read_text().strip()
    comm=Path('/proc')/pid/'comm'
    if comm.exists() and comm.read_text().strip()=='Xvfb':
        raise RuntimeError('display_91_already_owned_by_live_Xvfb')
    p.unlink()
    Path('/tmp/.X11-unix/X91').unlink(missing_ok=True)
PY
Xvfb :91 -screen 0 640x600x24 -nolisten tcp > artifacts/roboweave/xvfb.log 2>&1 &
XVFB_PID=$!
sleep 1
kill -0 "$XVFB_PID"
ros2 run turtlesim turtlesim_node > artifacts/roboweave/turtlesim.log 2>&1 &
TURTLE_PID=$!
python roboweave/motion_node.py > artifacts/roboweave/motion-service.log 2>&1 &
MOTION_PID=$!
if [ -f /run/secrets/share.env ]; then
  set -a
  source /run/secrets/share.env
  set +a
  python -m uvicorn roboweave.public_gateway:app --host 127.0.0.1 --port 8769 --no-access-log > artifacts/roboweave/gateway.log 2>&1 &
  GATEWAY_PID=$!
fi
cleanup(){
  kill "$MOTION_PID" "$TURTLE_PID" "$XVFB_PID" 2>/dev/null || true
  if [ -n "${GATEWAY_PID:-}" ]; then kill "$GATEWAY_PID" 2>/dev/null || true; fi
  if [ -n "${RUNTIME_PID:-}" ]; then kill "$RUNTIME_PID" 2>/dev/null || true; fi
}
trap cleanup EXIT INT TERM
python roboweave/runtime.py >> artifacts/roboweave/runtime.log 2>&1 &
RUNTIME_PID=$!
wait -n "$XVFB_PID" "$TURTLE_PID" "$MOTION_PID" "$RUNTIME_PID" ${GATEWAY_PID:+"$GATEWAY_PID"}
# Restart the whole isolated service if any essential child exits.
exit 1
