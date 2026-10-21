# Arm runtime: deployment and migration

## Verified deployment

Linux x86_64, RTX 4090, ROS 2 Jazzy in the existing RAI container image. The
simulator uses a project-local Python 3.10 `.venv-arm`, SAPIEN 3.0.3 and ManiSkill
3.0.1; RAI uses its existing Python 3.12 `.venv` and upstream `uv.lock`.
The simulator has CPU physics and GPU rendering. It uses privileged object pose,
not a learned perception model. Physics advances only while a skill is executing.

The deployed `.venv-arm` reuses reviewed ManiSkill dependencies through a `.pth`
reference to the existing benchmark environment, with NumPy 1.26.4 installed
locally. It does not modify that environment. Keep the referenced environment
available. A fresh standalone install recipe is provided but **a clean-machine
installation has not been verified**.

The unmodified official solver initially segfaulted at `mplib.Planner` with
NumPy 2.2.6. Switching only the isolated environment to NumPy 1.26.4 repaired
the same reproduction. This matches [upstream issue 1100](https://github.com/mani-skill/ManiSkill/issues/1100).

## Start the prepared workspace

```bash
cd /data/projects/rai-roboweave
bash scripts/start_roboweave_arm.sh
systemctl --user start roboweave-arm-tunnel
```

The startup script is idempotent and preserves existing containers. For a source
update, wait until `/api/roboweave/status` reports `busy=false`, then explicitly
restart only these application services:

```bash
systemctl --user restart roboweave-arm-sim
sudo docker restart rai-roboweave-arm
```

The action interface was built with:

```bash
source /opt/ros/jazzy/setup.bash
CC=gcc-14 CXX=g++-14 colcon build --packages-select roboweave_interfaces --symlink-install
```

For a new simulator environment, review dependencies first and run
`bash scripts/setup_arm_sim.sh`. Set `MANISKILL_SITE_PACKAGES` only when deliberately
reusing a reviewed existing installation; otherwise the pinned core dependencies
in `requirements-arm.txt` are installed into `.venv-arm`. The provided Dockerfile
builds RAI and the ROS interfaces; it does not install the separate host simulator.

## Processes, secrets and access

| Component | Private address / scope |
|---|---|
| `roboweave-arm-sim.service` | `127.0.0.1:8772`, single physics owner |
| RAI arm runtime | `127.0.0.1:8770` |
| Authenticated gateway | `127.0.0.1:8771` |
| `rai-roboweave-arm` container | ROS domain 74, localhost discovery |
| `roboweave-arm-tunnel.service` | HTTPS to the authenticated gateway |

DeepSeek and share credentials remain in read-only mounted files under
`/run/secrets/`; use the existing environment examples from the 2-D deployment.
No key is sent to the simulator, frontend, public evidence or GitHub. Only the
authenticated gateway is tunneled. Its allowlist exposes the dashboard, status,
real camera, task start and stop. It rejects missing authentication and foreign
Origins. The raw loopback APIs and ROS domain trust local processes; this is a
single-user demonstration, not a multi-tenant deployment.

The Quick Tunnel hostname is temporary and may change after its process restarts.
No permanent domain has been configured. Existing OpenETA and 2-D RAI services
remain separate and running.

## Repeat validation

On the host:

```bash
python3 scripts/verify_roboweave_arm.py --seed 0
python3 scripts/verify_roboweave_arm.py --seed 0 --fault
python3 scripts/verify_roboweave_arm.py --seed 1
```

Inside the prepared RAI container, with the project and ROS environments sourced:

```bash
export ROS_DOMAIN_ID=74 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python scripts/verify_arm_boundaries.py
python -m pytest -q roboweave/arm/test_runtime.py roboweave/arm/test_contracts.py roboweave/test_guards.py
```

Do not run these concurrently with another user's task. Receipts are saved to
`artifacts/arm/runs/<task_id>/` and `artifacts/arm/verification/`; every commanded
joint action and measured state is recorded in
`artifacts/arm/trajectories/<trajectory_id>/trajectory.jsonl`. PNG frames are
sampled every six control steps. This task reaches and holds the target; it does
not release the cube onto a surface.

## Hardware migration boundary

Keep `roboweave/arm/runtime.py`, ordered-plan enforcement, tool results, failure
latch and event logging. Replace `roboweave/arm/ros_adapter.py` and its loopback
ManiSkill backend with the robot's own ROS 2 driver / MoveIt integration.

The `Manipulate.action` contract carries skill, semantic object, coordinate frame,
deadline and typed TCP/object feedback. The current task uses `cube` and `world`;
the target is the configured goal published in `/roboweave/arm/state`. Reset seed
and fault injection are simulator-only fields and must be rejected or disabled
on hardware. The JSON observation must retain explicit source, timestamp, joint
positions, measured contact and goal verification. Define a calibrated goal-pose
contract when expanding beyond this single semantic task.

Required before hardware use: perception instead of privileged state, calibrated
frames/TF, reachability and collision checks, payload/gripper limits, independent
emergency stop, command watchdog, safe holding behavior and measured postcondition
thresholds. Simulator cancellation stops physics advancement; it **does not prove
a hardware stop controller**. No hardware, ARM/Jetson, offline inference or general
long-horizon manipulation validation has been performed.
