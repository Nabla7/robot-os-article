# Building a Software Stack for the Unitree G1 Humanoid

*Pim Van den Bosch*

---

The Unitree G1 is a $16,000 humanoid robot with 29 degrees of freedom. Out of the box, it walks. It waves. It streams sensor data. What it cannot do is see objects, navigate to them, understand speech, or learn new tasks. The gap between a robot that balances and a robot that does useful work is enormous, and largely a software problem.

Between September 2025 and March 2026, I built Robot-OS — the software that bridges that gap. It gives the G1 perception, autonomous navigation, voice interaction, and learned manipulation, running across two machines connected by WiFi. Hundreds of commits, roughly six months of evenings and weekends, a lot of broken hardware.

This is a description of that system: what it does, how the pieces fit together, and the parts that took longer than they should have.

## What the robot ships with

The G1 runs a reinforcement-learning locomotion policy on its firmware. This handles walking, balance, and recovery from pushes. The firmware also includes a gesture system — pre-programmed motions like handshakes, claps, and waves — accessible through a C++ API. There is a Jetson Orin NX mounted inside the torso, a Livox MID-360 LiDAR on the head, and an Intel RealSense D435 depth camera.

The locomotion policy is good. I never had to touch it. Everything I built sits on top of it.

## The architecture, honestly

The system is not a monolith and it's not a microservice architecture. It's what happens when you need SAM 3 (segmentation), FoundationPose (6-DoF pose estimation), ROS2, a voice model, and an IK solver to coexist, and they all require incompatible Python environments.

`start_workstation.sh` launches roughly ten processes:

- A **FastAPI backend** (port 8000) that hosts the coordination logic: navigation routing, the routine state machine, voice relay, arm mode switching, and the API endpoints the frontend talks to. This is the closest thing to a "core" — services inside it share state through Python singletons and asyncio events.
- **Spatial memory** (port 8090) — a separate FastAPI app with its own conda environment, responsible for storing and querying detected objects. The main backend talks to it over HTTP.
- **SAM 3** (port 8091), **SAM 3D** (port 8092), **FoundationPose** (port 8093) — each a separate process in its own conda environment, each called by the spatial memory service over HTTP.
- **ACT skill runner** (port 8098) — runs learned manipulation policies in the `unitree_lerobot` conda environment.
- **XR teleop server** (port 8012) — HTTPS WebSocket server for VR headset connections, runs Pink IK in the `teleop` conda environment.
- **Nav→Rerun bridge**, **joint state visualizer** — separate processes that stream data to the Rerun 3D viewer.
- **React frontend** (port 5173).

The reason for this fragmentation is entirely practical: SAM 3 needs PyTorch with specific CUDA versions, FoundationPose needs a different set of dependencies, the voice runtime has its own version constraints, and Pink IK needs Pinocchio which conflicts with half the perception stack. Conda environments can't be shared. So each capability runs in its own process, and they talk over HTTP and WebSocket on localhost.

What makes this different from a typical ROS graph is where the coordination lives. In ROS, multi-step tasks (navigate → detect → grasp) require an orchestration layer on top of the message bus. In Robot-OS, the routine executor is a Python state machine inside the FastAPI backend that dispatches steps sequentially: it sends a WebSocket message to the Jetson nav stack for navigation, makes an HTTP POST to the motion proxy for gestures, and calls the voice service API for speech — then waits for completion events before advancing. The state machine, the waiting logic, and the error handling are all in one file (`routine_executor.py`), not spread across a node graph.

The voice agent works the same way. When you say "go to the red cup," the LLM generates a tool call. The `action_executor.py` in the voice service makes an HTTP GET to `http://127.0.0.1:8000/spatial-memory/search/objects?query=red+cup`, gets back coordinates, and POSTs a navigation goal to the backend's `/navigation/goto` endpoint. It's HTTP calls, not shared memory. But the control flow is explicit and sequential — you can read it top to bottom — which made debugging significantly easier than chasing messages through a pub-sub graph.

The trade-off is real: swapping out the perception backend means changing HTTP endpoints and response formats. There's no standardized message type like ROS provides. But for a system where every component was changing weekly, having the integration logic in readable Python with `httpx` calls was more productive than maintaining message definitions and launch files.

The one piece that uses ROS is the navigation stack, which runs ROS2 inside a Docker container on the Jetson. It's isolated because the ROS2 dependency tree is massive and because the nav stack was originally packaged that way. Nothing else in the system uses ROS.

### Two machines

These ten processes all run on the workstation. But the system spans two machines: the workstation across the room, and the Jetson Orin NX mounted inside the robot. The Jetson handles sensor relay and navigation. The workstation handles everything else.

This split exists for two reasons. First, the Jetson doesn't have the compute to run SAM 3, FoundationPose, and a voice model simultaneously — those are GPU-hungry inference workloads that need a proper workstation GPU. Second, and more fundamentally, the robot's internal communication bus is unreachable from outside.

![System Architecture](architecture.png)

### The DDS subnet problem

The G1's motors and sensors sit on a DDS (Data Distribution Service) bus locked to a 192.168.123.x subnet. DDS is a pub-sub middleware where nodes discover each other by broadcasting their network addresses — it's the standard communication layer for ROS2 and for Unitree's firmware. This subnet is only accessible from the Jetson's internal ethernet interface — it's the physical bus that connects to the motor controllers, IMU, and LiDAR inside the robot's body. The workstation lives on a different subnet (192.168.0.x), reachable over WiFi.

The obvious approach is IP forwarding: configure the Jetson as a gateway, route packets from the WiFi subnet to the internal bus, done. I spent a day on this. It doesn't work. The DDS implementation (Cyclone DDS, used by Unitree) embeds the sender's IP address in the protocol payload itself. When a node on the wrong subnet sends a message, DDS peer discovery rejects it — the embedded IP doesn't match the expected subnet, so the message is silently dropped, even if it was correctly routed at the IP level. This isn't a misconfiguration — it's a fundamental property of how DDS works.

### Relay processes

The solution is relay processes. Six Python scripts run on the Jetson host (outside Docker), each bridging one data stream between the DDS bus and a WebSocket or HTTP endpoint the workstation can reach:

- **Arm command relay** (port 8097): receives joint position targets from the workstation over WebSocket, publishes them to the `rt/arm_sdk` DDS topic at 250 Hz. This is the most complex relay — a multi-threaded control loop with PD gain management, velocity limiting, gesture release logic, and a 300-second timeout-to-release mechanism. It's the critical path for teleoperation and policy execution, and the most safety-critical code in the system.

- **Camera relay** (port 8090 on the Jetson): captures RGB-D frames from the RealSense D435, JPEG-encodes RGB, PNG-encodes depth (lossless uint16 to preserve precision), base64-wraps both, and streams them over WebSocket. The workstation's perception pipeline consumes these frames for object detection and pose estimation. (The spatial memory service also runs on port 8090, but on the workstation — different machines, same port number, a source of confusion I never bothered to fix.)

- **Lowstate relay** (port 8096): reads 29 joint positions off the DDS bus via raw multicast and forwards them at 30 Hz over WebSocket. This feeds the Rerun visualization (live URDF rendering) and provides the observation input for ACT policy inference.

- **Velocity command relay** (port 8098): receives teleop velocity commands from the workstation over WebSocket, forwards them to the SDK's `Move()` function for manual driving.

- **Velocity command executor** (no listening port — it's a WebSocket *client* that connects to the nav stack's port 9002): receives `cmd_vel` navigation commands from the ROS2 path follower inside the Docker container, and calls the Unitree SDK's `Move()` function to actually make the robot walk. Without this bridge, the navigation stack can plan paths but the robot won't move.

- **Motion proxy** (port 8095): receives gesture trigger requests over HTTP REST, dispatches them to the firmware's gesture system via the `G1ArmActionClient` DDS interface. Fire and forget — separated because gesture execution runs at firmware level with full motor authority, distinct from the `arm_sdk` overlay.

Each relay is independent. If the camera relay crashes, navigation still works. If the arm relay is down, the voice agent can still dispatch gestures through the motion proxy. This fault isolation wasn't a design goal — it fell out of the architecture naturally — but it turned out to be one of the system's best properties.

### The workstation stack

A React frontend provides monitoring and control: live camera feeds, a 2D navigation map with waypoint placement, gesture triggers, voice controls, and sensor readouts. Rerun renders a 3D view with the robot's URDF model at its estimated position, overlaid on the accumulated LiDAR pointcloud.

## Perception

The perception pipeline detects objects in the camera feed, estimates their 3D pose, and stores them on the SLAM map. It runs as a chain of HTTP calls between four separate processes, each in its own conda environment.

The backend captures an RGB-D frame from the camera relay and the robot's current pose from the navigation WebSocket, then POSTs both to the spatial memory service (port 8090). From there:

1. **SAM 3** (port 8091) receives the RGB frame and returns segmentation masks and bounding boxes — 2D detection only.
2. **SAM 3D** (port 8092) takes each mask along with the depth image and reconstructs a coarse 3D mesh (`.obj`). This is the step that lifts detection from 2D to 3D.
3. **FoundationPose** (port 8093) takes each mesh and estimates its 6-DoF pose in the robot's coordinate frame.

When an object is stored, the spatial memory service also runs CLIP to encode it as a vector embedding — this enables text search ("find the red mug") but it's a post-processing step inside the storage logic, not a separate service call.

The spatial memory service stores everything — object positions, orientations, meshes, embeddings — anchored to the robot's pose at detection time.

Only one model runs on the GPU at a time. The spatial memory service enforces strict single-stage GPU ownership: it unloads both FoundationPose and SAM 3 before running SAM 3D (pass 1: mesh reconstruction), then unloads SAM 3D before reloading FoundationPose (pass 2: pose estimation). This orchestration means a single workstation GPU can handle the full pipeline without running out of VRAM — it just takes longer per object.

The detail that made this pipeline feasible: SAM 3D's coarse mesh reconstructions are good enough for FoundationPose. Most FoundationPose workflows require pre-scanned CAD models of each object. Using SAM 3D's output instead closes the loop from open-vocabulary detection to 6-DoF localization without knowing what the objects are in advance. I first used FoundationPose during an internship at VUB's BruBotics lab, and the realization that it could work with low-quality meshes was what made the whole spatial memory concept possible.

The pipeline has two unsolved problems:

**Deduplication.** The same mug seen from three angles creates three separate entries. Merging detections across viewpoints requires matching by position and appearance, and the thresholds that work for large objects fail for small ones.

**Identity instability.** An open-vocabulary detector will label an object "coffee mug" once, "ceramic cup" the next time, and segment half the table as an object on the third pass. This inconsistency accumulates over time. Constraining the detector to an expected object list helps, but defeats the purpose of open-vocabulary detection.

These reflect a real gap between single-image detection — which current models handle well — and the persistent spatial inventory a robot actually needs.

![Spatial Memory Visualization](spatial-memory.jpg)

## Navigation

Autonomous navigation uses a ROS2 stack running in a Docker container on the Jetson. It fuses LiDAR scans with IMU data for SLAM, computes costmaps, plans paths, and outputs velocity commands.

Two WebSocket channels connect the nav stack to the workstation: one for visualization data (point cloud, robot pose, planned path) and one for velocity commands that the executor forwards to the robot's `Move()` function.

The integration between ROS2's path follower and Unitree's locomotion SDK was non-trivial. The navigation stack outputs `cmd_vel` velocity commands inside the Docker container, but the Unitree SDK can only be called from the Jetson host. The `cmdvel_ws_executor.py` bridge process connects to the container's WebSocket on port 9002, receives velocity commands, and translates them into SDK `Move()` calls. I debugged this chain for an entire weekend before realizing the executor was disabled by a single environment variable (`NAV_EXECUTOR_ENABLED=0`).

The frontend lets you place waypoints on a 2D map, and the robot plans paths to them autonomously, including correct yaw orientation at the destination. The voice agent can also trigger navigation: "Go to the bottle" queries spatial memory for the bottle's position and sends it as a navigation goal. An emergency stop flag in the navigation executor can block all movement instantly — the one safety mechanism I'm glad I never had to use in a real emergency.

Visualization bandwidth was a problem. The raw WebSocket broadcast from the nav stack was 6.5 MB per message at 1 Hz — mostly navigation graph and polygon markers that nobody needed. Disabling those fields dropped it to 941 KB at 4 Hz, which made the Rerun pointcloud visualization actually usable.

![Navigation Visualization](navigation.jpg)

## Voice control

A voice interface connects the robot to OpenAI's Realtime API for speech-to-speech conversation. The robot listens through a USB microphone on the workstation (the Jetson's audio hardware is limited), and the LLM generates both spoken responses and tool calls.

Available tools include: navigate to a location, trigger a gesture, query spatial memory, capture and describe what the robot sees (the camera frame goes to GPT-4V, the description is spoken back), and activate a manipulation skill. "What do you see?" triggers the camera, runs vision analysis, and speaks the result — the robot has a visual perception loop accessible through natural language.

A routine executor can chain these into multi-step plans — "go to the kitchen, find the mug, pick it up" becomes a sequence of navigation goals, perception queries, and skill activations, orchestrated by a state machine that waits for each action to complete before advancing.

Single-step commands work reliably. Multi-step plans compound errors. If perception, navigation, and manipulation each succeed 85% of the time, a three-step chain succeeds 61% of the time. In practice, the rates are often lower. A navigation goal reached 30 cm off target means the subsequent grasp fails, and the planner doesn't know why.

The bottleneck is not the language model. Current LLMs plan well enough for this kind of task. The bottleneck is the state representation they receive. The planner can only act on what the perception system actually detected, which may not reflect what's in the room.

The voice service itself has more infrastructure than you'd expect: a personality system with loadable configs (different system prompts, voices, and behaviors), conversation persistence across sessions, VAD via WebRTC for wake word detection, and a supervisor process that auto-restarts the voice runtime with exponential backoff if it crashes.

![Voice Interaction](voice-conversation.jpg)

## The arm control interface nobody documented

Before getting to the teleoperation story, the firmware interface needs explaining — because this is where most of the debugging time went.

The Unitree SDK documentation covers the basics. What it doesn't cover is what actually matters when you're trying to do manipulation.

The firmware exposes two DDS topics for motor control:

**`rt/lowcmd`** gives you direct authority over every joint. Position targets, velocity targets, torque, PD gains — all 29 degrees of freedom. The locomotion policy stops running. You control the legs. If your commands are wrong, the robot falls.

**`rt/arm_sdk`** is the overlay channel. You send commands for the arm and waist joints (indices 12–28, 17 joints total) while the locomotion policy continues to handle the legs. The firmware blends your commands with its own using a weight parameter stored in `motor_cmd[29].q`, where 1.0 means full external control. In practice, the IK solver only controls the 14 arm joints (7 per arm) and locks the 3 waist joints at their startup position — more on that in the teleoperation section.

This is the mode you want for manipulation. The robot keeps walking and balancing while you move the arms independently. The gesture system and teleop system can't run simultaneously — an arm mode router (`arm_mode.py`) enforces mutual exclusion between gesture, slider, and teleop control.

You must create a fresh message object every publish cycle. Reusing a stale `LowCmd` object causes the firmware to weaken or ignore arm commands. The Unitree SDK examples always create a new message each tick. This is not an optimization choice.

You must set kp, kd, q, dq, and tau for every arm and waist joint. Omitting joints doesn't mean "leave them alone." It means "command them to their default values." Setting PD gains on leg joints in an `arm_sdk` message causes the firmware to reject the entire message.

The gesture system runs on a completely separate firmware channel. Gestures produce stiff, forceful motions because they run with full motor authority inside the firmware. The `arm_sdk` overlay has lower torque limits. I measured about 8 Nm of effective shoulder torque through `arm_sdk` with kp=80 — enough to lift the arm overhead, but noticeably weaker than a gesture doing the same motion.

None of this is in the documentation. None of it is in the GitHub issues. Each detail cost hours.

## Teleoperation: the twelve-hour bug

Before the robot can learn manipulation, it needs training data. Before you can collect training data, you need working teleoperation. This turned out to be the hardest part of the project.

The setup: a Pico VR headset streams controller poses over WebSocket to the workstation. An IK solver converts 6-DoF wrist targets into 14-DoF joint angles. These are sent to the Jetson's arm relay, which publishes them to `rt/arm_sdk`.

The IK solver is Pink, a QP-based velocity IK library built on Pinocchio. It replaced a CasADi/IPOPT solver that was in the original codebase. Pink computes incremental velocity steps toward the target rather than re-solving the full nonlinear problem each frame. The difference: 0.5 ms per step vs 5-15 ms, and much less jitter. It also supports per-joint posture weights, which keep the robot's arm configuration natural instead of drifting into kinematically valid but physically awkward poses.

A 0.7× workspace scaling maps human arm reach to the robot's shorter arms. Without it, extending your arm to 70% of your reach already pushes the robot to its kinematic limits.

After the IK solver, transforms, and scaling were all working correctly, the arms still couldn't lift above chest height. I could command the shoulder pitch to a position above the head. The arm would barely move. But pushing the arm by hand revealed it was compliant — it would hold a position if placed there, then slowly drift back. The motors had authority. Something was preventing them from using it.

I spent twelve hours on this. I replaced the IK solver. Added feed-forward gravity compensation torques. Tuned PD gains. Restructured the coordinate pipeline. Every change was correct. None had any visible effect.

The cause was a velocity clipper buried in the arm relay. The relay's control loop runs at 250 Hz. Each tick, the clipper enforced a velocity limit of 5.0 rad/s — which at 250 Hz means the commanded position could move at most 0.02 radians per tick from the motor's current position. The intent was smooth motion. The effect was that the PD controller never saw more than a tiny position error, so it never generated more than a fraction of available torque. The arm couldn't overcome gravity because the relay was preventing it from trying.

I removed the clipper. The arm lifted above the head on the first try.

## Learning manipulation

With teleoperation working, the pipeline is: collect demonstrations through VR, convert to training format, train a policy, deploy it.

I collected 50 episodes of a pick task using rubber grippers (the Dex3 hands had broken during earlier teleop sessions — no force feedback means you can't feel when fingers hit a surface). Each episode records the head camera feed and the full 29-DoF joint state at 50 Hz — though only 14 of those joints (the arms) are operator-commanded. The remaining 15 (legs and waist) are recorded from the firmware's locomotion policy, providing context for the policy to learn in.

The recordings convert to LeRobot format for training an ACT (Action Chunking with Transformers) policy. Given a camera image and current joint state, the policy predicts a chunk of future joint positions. 52 million parameters — ResNet18 vision backbone plus transformer decoder. Training on 50 episodes for 100k steps takes about 45 minutes on an RTX 5090. Loss drops from 7.7 to below 0.1.

Deployment uses the same arm relay as teleoperation. The skill runner moves the arms to a start pose, then runs inference at 50 Hz: read camera, read joints, predict action chunk, send targets. The policy runs until a termination condition or timeout.

The Dex3 hands deserve a mention. They're mechanically capable but not designed for this kind of abuse. Multiple fingers broke during data collection. When a thumb pin snapped, I measured it, modeled a replacement in CAD, and had a local CNC shop machine new ones. The hands are not designed to be disassembled. You learn the internals by breaking them.

## What the system looks like end to end

In the demo configuration, a user says "pick up the red mug." Here's the actual call chain:

1. The voice agent (OpenAI Realtime API) generates a `navigate_to_object` tool call. `action_executor.py` makes an HTTP GET to the backend's spatial memory search endpoint with `query=red+mug`.
2. The backend proxies this to the spatial memory service (workstation:8090), which runs a CLIP similarity search and returns the object's stored position and the robot's pose when it was detected.
3. `action_executor.py` POSTs a navigation goal to `/navigation/goto`. The backend sends a WebSocket message to the Jetson nav stack (port 9001) with target coordinates.
4. The nav stack plans a path and streams `cmd_vel` to the executor (port 9002), which calls `Move()` on the SDK. The robot walks to the target.
5. On arrival, the voice agent activates the pick policy. The ACT skill runner (port 8098) reads camera frames and joint state, runs inference at 50 Hz, and sends arm targets through the arm relay (port 8097) to `rt/arm_sdk`.

Each step works individually. The chain breaks at the seams: navigation arrives 30 cm off target and the grasp fails. Spatial memory returns the object's position from five minutes ago and it's been moved. The pick policy trained on rubber grippers doesn't generalize to a slightly different cup orientation. The voice agent doesn't know any of this happened — it gets back `{ok: true}` from the navigation endpoint regardless of arrival accuracy.

This is the expected state of a system where every component is early-stage and the interfaces between them are thin. What the system demonstrates is the structure: what layers a humanoid robot OS needs, how they connect, and where the failure modes live.

## Three things I'd change

**Define service interfaces before writing services.** The HTTP interfaces between the ten processes were designed after the fact, which means they're inconsistent. Some return JSON with an `ok` field, some don't. Some use WebSocket, some use REST. Error handling varies per service. Defining a contract up front — even something as simple as "every service returns `{ok, data, error}`" — would have saved weeks of integration debugging.

**Build a proper spatial memory layer.** The current system stores object positions at detection time and never updates them. Objects move. The robot moves. A useful spatial inventory needs to track objects over time, merge observations, and handle uncertainty. This is a well-studied problem in robotics. Integrating it with foundation-model detectors is not.

**Interpolate between teleop commands.** The VR headset sends targets at 30 Hz. The arm relay runs at 250 Hz. Sending raw IK output directly to the motors produces visible vibration. A simple interpolation buffer would smooth this, improving both the teleoperation experience and the quality of collected training data.

---

*Pim Van den Bosch — robotics, simulation, whole-body control. Based in Belgium.*
