# RAI / ROS 2 vertical slice

User target: no physical robot is currently available. Demonstrate actual ROS 2 skill calls, execution feedback, cancellation, and bounded failure recovery with a runtime whose robot adapter can be replaced later. The application does not replace unrelated deployments.

Baseline: upstream `examples/agents/react_ros2.py`, unmodified, on ROS 2 Jazzy in an isolated container; model configuration uses the existing server-side DeepSeek credential. Record incoming HRI message, real toolkit calls, native RotateAbsolute action and resulting Pose. The full O3DE Panda demo requires a separate binary and multimodal model; the selected baseline does not claim Panda manipulation.

Increment: a ROS2 MoveToPose action (goal, feedback, result, cancel) implemented by a turtlesim Twist/Pose adapter; RAI's original `create_react_runnable` and `ROS2Connector`; strict fresh-pose verifier; at most one retry after an explicitly aborted injected transient failure. No retry for unknown effects, cancellation, timeout, or stale observations. User coordinate sequence must match successful skill sequence for task success. Record JSONL events, agent messages, pose trajectory, final result. The UI reads actual state and the actual native simulator window.

Acceptance: official entry executed; at least one two-goal task; fault abort plus bounded successful recovery; cancellation stops velocity; invalid goal rejected; stale observations fail closed; authenticated external HTTPS page shows the running simulator and receipts. Hardware/ARM/offline inference and manipulation are unverified.

No OpenSpec executable was found during server audit. This document is the bounded change specification. Repository AGENTS.md requires Jazzy and g++14; both are isolated inside the new container. No ROS installation or driver changes to the Ubuntu 22.04 host. ROS domain 73 and localhost discovery isolate this demo from other robot networks.
