# Run and verify RoboWeave

## Prepared workspace: executed and verified

The cloud installation has ROS 2 Jazzy, g++14, Python 3.12, project-local `.venv`, and compiled `rai_interfaces` / `roboweave_interfaces`. `config.toml` uses the original OpenAI-compatible provider with `deepseek-chat` and `https://api.deepseek.com/v1`.

From `/rai` inside the prepared container:

```bash
source /opt/ros/jazzy/setup.bash
export SHELL=/bin/bash
source setup_shell.sh
python -m pytest -q roboweave/test_guards.py tests/communication/ros2/test_get_future_result_timeout.py tests/tools/ros2/test_input_validation.py tests/tools/ros2/test_nav2_bounds.py tests/communication/ros2/test_exceptions.py
```

This exact test selection passed 60 tests. To exercise the live simulator while no other task is running:

```bash
python scripts/verify_ros_boundaries.py
python scripts/verify_roboweave_rai.py
```

The latter uses the configured LLM API and incurs provider usage. It runs ordered navigation, one injected fault with bounded recovery, and cancellation. The native boundary test moves the turtle and tests rejection / deadline handling.

The process entry is `bash scripts/run_roboweave_rai.sh`. It expects `/run/secrets/deepseek.env`; the optional `/run/secrets/share.env` enables the authenticated gateway. Only server-side processes receive the model key. No keys belong in HTML, Git, images or logs.

## Clean image build: recipe provided, fresh-host rebuild not yet verified

The Docker recipe reproduces the package and build commands used for the working environment. Network access to Ubuntu/ROS/Python registries is required. An additional clean-host build is still unverified; the installed-environment runtime evidence must not be interpreted as a clean-image build result.

On Linux with Docker:

```bash
git clone https://github.com/Benjamindaoson/robo-weave.git
cd robo-weave
mkdir -p secrets
cp config-examples/deepseek.env.example secrets/deepseek.env
cp config-examples/share.env.example secrets/share.env
chmod 700 secrets
chmod 600 secrets/*.env
# Edit the two files locally. Replace every placeholder.
docker build -f docker/Dockerfile.roboweave -t roboweave-rai:source .
docker run -d --name roboweave-rai-source --restart unless-stopped --network host \
  --mount "type=bind,src=$PWD/secrets/deepseek.env,dst=/run/secrets/deepseek.env,readonly" \
  --mount "type=bind,src=$PWD/secrets/share.env,dst=/run/secrets/share.env,readonly" \
  --mount type=volume,src=roboweave-rai-evidence,dst=/rai/artifacts/roboweave \
  roboweave-rai:source
```

Choose unused ports or adjust the code before running alongside an existing instance: runtime is loopback 8768, authenticated gateway is loopback 8769, ROS domain is 73 with localhost discovery. The prepared-workspace `start_roboweave_rai_container.sh` is for the existing deployment and its mounted `.venv`; the self-contained image command above does not mount a source directory over `/rai`.

For browser use, put an HTTPS reverse proxy or tunnel in front of **8769**. The login cookie is Secure; browser authentication is designed for HTTPS. Do not expose **8768**, DDS traffic, or a general-purpose ROS tool endpoint publicly. A private SSH tunnel to 8768 is suitable for local development.

## Logs and operation

`artifacts/roboweave/runs/<task-id>/` contains events, Agent messages, actual position trajectory and final results. The parent directory contains runtime, gateway, simulator and motion-adapter logs. API status is `/api/roboweave/status`; task start and cancellation use `/api/roboweave/start` and `/api/roboweave/stop`.

The startup script supervises the simulator, display server, motion adapter, runtime and optional gateway. It checks the container-private X11 lock after restart. Restart policies recover processes; interrupted robot tasks are **not** resumed automatically. An interrupted run can have event files without a final result. The UI supports one active task at a time.

For the official baseline, run the unchanged `examples/agents/react_ros2.py` under the same sourced ROS environment and model credentials, then run `scripts/official_probe.py` from a second shell. Stop that official agent afterward to avoid multiple motion authorities.

## Verification boundaries

CI checks source syntax only. ROS execution, LLM tool calls and fault recovery were separately verified on the cloud environment. Full upstream optional extras, full upstream suite, clean-host image build, ARM/Jetson, physical hardware, local inference and manipulation remain unverified. The planner/driver split supports later integration; it does not establish physical robot safety.
