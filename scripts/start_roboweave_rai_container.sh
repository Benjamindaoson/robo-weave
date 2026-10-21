#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
NAME=rai-roboweave-runtime
if sudo docker container inspect "$NAME" >/dev/null 2>&1; then
  sudo docker start "$NAME"
else
  sudo docker run -d --name "$NAME" --restart unless-stopped --network host \
    --env ROS_DOMAIN_ID=73 --env ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    --mount "type=bind,src=$PWD,dst=/rai" \
    --mount "type=bind,src=${DEEPSEEK_ENV_FILE:-/home/linux/.config/deepseek/api-key.env},dst=/run/secrets/deepseek.env,readonly" \
    --mount "type=bind,src=${SHARE_ENV_FILE:-/home/linux/.config/roboweave/share.env},dst=/run/secrets/share.env,readonly" \
    --workdir /rai roboweave-rai:20261021 bash scripts/run_roboweave_rai.sh
fi
