# RAI arm manipulation slice

Replace the primary 2-D demonstration with a separate Panda / ManiSkill PickCube
runtime. Preserve the original ROS 2 demo and OpenETA services. Reuse the installed
ManiSkill environment without changing its dependencies. Physics stays in one
thread; RAI runs in its existing ROS 2 Jazzy container environment.

Acceptance: upstream PickCube solver baseline; RAI tool calls transported through
a real ROS 2 Action; contact-verified grasp and native environment success;
physically open-gripper fault, failed grasp postcondition, one bounded recovery;
cancel/deadline/invalid command handling; real camera, feedback and persisted
state/action trajectory; authenticated HTTPS demo and sanitized GitHub evidence.

The simulator uses privileged object state and the official motion planner. This
is not a vision policy, learned VLA, hardware validation, or independent LLM
multi-agent team. Planner is RAI ReAct; execution, verifier and recovery are
deterministic roles. Hardware migration replaces the action adapter, with new
calibration, perception, collision checking and safety validation required.
