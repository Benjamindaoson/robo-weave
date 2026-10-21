#!/usr/bin/env bash
set -e
cd /rai
source /opt/ros/jazzy/setup.bash
export SHELL=/bin/bash
source ./setup_shell.sh
set -a
source /run/secrets/deepseek.env
source /run/secrets/share.env
set +a
export OPENAI_API_KEY="$DEEPSEEK_API_KEY"
export ROS_DOMAIN_ID=74
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROBOWEAVE_UPSTREAM=http://127.0.0.1:8770
mkdir -p artifacts/arm
python -m roboweave.arm.ros_adapter > artifacts/arm/adapter.log 2>&1 &
ADAPTER_PID=$!
python -m roboweave.arm.runtime > artifacts/arm/runtime.log 2>&1 &
RUNTIME_PID=$!
python -m uvicorn roboweave.public_gateway:app --host 127.0.0.1 --port 8771 --no-access-log > artifacts/arm/gateway.log 2>&1 &
GATEWAY_PID=$!
cleanup(){ kill "$ADAPTER_PID" "$RUNTIME_PID" "$GATEWAY_PID" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
wait -n "$ADAPTER_PID" "$RUNTIME_PID" "$GATEWAY_PID"
exit 1
