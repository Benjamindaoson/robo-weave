#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
test -x .venv-arm/bin/python
test -d install/roboweave_interfaces
mkdir -p "$HOME/.config/systemd/user" artifacts/arm
cat > "$HOME/.config/systemd/user/roboweave-arm-sim.service" <<EOF
[Unit]
Description=RoboWeave isolated Panda physics
After=network-online.target
[Service]
WorkingDirectory=$PWD
Environment=OMP_NUM_THREADS=1
Environment=OPENBLAS_NUM_THREADS=1
ExecStart=$PWD/.venv-arm/bin/python -u -m roboweave.arm.sim_worker
Restart=on-failure
RestartSec=5
LimitCORE=0
[Install]
WantedBy=default.target
EOF
systemctl --user daemon-reload
systemctl --user enable --now roboweave-arm-sim
NAME=rai-roboweave-arm
if sudo docker container inspect "$NAME" >/dev/null 2>&1; then
  sudo docker start "$NAME"
else
  sudo docker run -d --name "$NAME" --restart unless-stopped --network host \
    --env ROS_DOMAIN_ID=74 --env ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    --mount "type=bind,src=$PWD,dst=/rai" \
    --mount "type=bind,src=${DEEPSEEK_ENV_FILE:-/home/linux/.config/deepseek/api-key.env},dst=/run/secrets/deepseek.env,readonly" \
    --mount "type=bind,src=${SHARE_ENV_FILE:-/home/linux/.config/roboweave/share.env},dst=/run/secrets/share.env,readonly" \
    --workdir /rai roboweave-rai:20261021 bash scripts/run_roboweave_arm.sh
fi
