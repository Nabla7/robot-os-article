# Building a Software Stack for the Unitree G1 Humanoid

*Pim Van den Bosch*

---

The Unitree G1 is a $16,000 humanoid robot with 29 degrees of freedom. Out of the box, it walks. It waves. It streams sensor data. What it cannot do is see objects, navigate to them, understand speech, or learn new tasks. Between September and November 2025, I built the software that adds those capabilities.

This is a description of that system: what it does, how the pieces fit together, and the parts that took longer than they should have.

## What the robot ships with

The G1 runs a reinforcement-learning locomotion policy on its firmware. This handles walking, balance, and recovery from pushes. The firmware also includes a gesture system — pre-programmed motions like handshakes, claps, and waves — accessible through a C++ API. There is a Jetson Orin NX mounted inside the torso, a Livox MID-360 LiDAR on the head, and an Intel RealSense D435 depth camera.

The locomotion policy is good. I never had to touch it. Everything I built sits on top of it.

## The architecture

The system spans two machines. The Jetson inside the robot handles sensor relay and navigation. A workstation across the room handles everything else: the web frontend, the backend services, perception inference, voice processing, and policy training. They communicate over WiFi.

This split exists because the Jetson doesn't have the compute to run SAM3, FoundationPose, and a voice model simultaneously. It also exists because the robot's internal communication bus is unreachable from outside.

![System Architecture](architecture.png)

The G1's motors and sensors sit on a DDS bus locked to a 192.168.123.x subnet. This subnet is only accessible from the Jetson's internal network interface. The workstation lives on a different subnet (192.168.0.x), reachable over WiFi. I spent a day trying to bridge these with iptables rules and IP forwarding on the Jetson. It doesn't work. The DDS implementation embeds the sender's IP in the protocol payload, so messages from the wrong subnet get silently dropped.

The solution is relay processes. Five small Python scripts run on the Jetson, each bridging one data stream between the DDS bus and a WebSocket or HTTP endpoint the workstation can reach:

- **Arm command relay**: receives joint targets from the workstation, publishes to `rt/arm_sdk` at 250 Hz
- **Camera relay**: captures frames from the RealSense D435, streams them over WebSocket
- **Lowstate relay**: reads joint positions off the DDS bus, forwards them at 30 Hz
- **Velocity command executor**: receives navigation velocity commands, calls the Unitree SDK's `Move()` function
- **Motion proxy**: receives gesture trigger requests over HTTP, dispatches them to the firmware

This is not elegant, but it works, and it made the rest of the system possible. Each relay is independent. If the camera relay crashes, navigation still works. If the arm relay is down, the voice agent can still dispatch gestures through the motion proxy.

## The arm control interface nobody documented

Most of the time I spent on this project was fighting the interface between my software and the robot's motors. The Unitree SDK documentation covers the basics. What it doesn't cover is what actually matters when you're trying to do manipulation.

The firmware exposes two DDS topics for motor control:

**`rt/lowcmd`** gives you direct authority over every joint. Position targets, velocity targets, torque, PD gains — all 29 degrees of freedom. The locomotion policy stops running. You control the legs. If your commands are wrong, the robot falls.

**`rt/arm_sdk`** is the overlay channel. You control the arm and waist joints (indices 12–28) while the locomotion policy continues to handle the legs. The firmware blends your commands with its own using a weight parameter stored in `motor_cmd[29].q`, where 1.0 means full external control.

This is the mode you want for manipulation. The robot keeps walking and balancing while you move the arms independently.

Three things I learned through experimentation:

**You must create a fresh message object every publish cycle.** Reusing a stale `LowCmd` object causes the firmware to weaken or ignore arm commands. The Unitree SDK examples always create a new message each tick. This is not an optimization choice.

**You must set kp, kd, q, dq, and tau for every arm and waist joint.** Omitting joints doesn't mean "leave them alone." It means "command them to their default values." Setting PD gains on leg joints in an `arm_sdk` message causes the firmware to reject the entire message.

**The gesture system runs on a completely separate firmware channel.** Gestures produce stiff, forceful motions because they run with full motor authority inside the firmware. The `arm_sdk` overlay has lower torque limits. I measured about 8 Nm of effective shoulder torque through `arm_sdk` with kp=80 — enough to lift the arm overhead, but noticeably weaker than a gesture doing the same motion.

None of this is in the documentation. None of it is in the GitHub issues. Each detail cost hours.

## Perception

The perception pipeline detects objects in the camera feed, estimates their 3D pose, and stores them on the SLAM map. Three models work in sequence:

**SAM3** takes an RGB frame and produces segmentation masks and coarse 3D meshes for each detected object. Mesh quality varies. Mugs and bottles reconstruct well. Irregular shapes usually don't.

**FoundationPose** takes each mesh and a depth image and estimates the object's 6-DoF pose. I first used FoundationPose during an internship at VUB's BruBotics lab. The insight here was that SAM3's mesh output could feed directly into FoundationPose's input, closing the loop from detection to localization without a separate CAD model.

**CLIP** encodes each detection as a vector embedding, stored alongside the object's map position. This enables text search from the frontend: type "red mug" and the system highlights the closest match.

The pipeline has two unsolved problems:

**Deduplication.** The same mug seen from three angles creates three separate entries. Merging detections across viewpoints requires matching by position and appearance, and the thresholds that work for large objects fail for small ones.

**Identity instability.** An open-vocabulary detector will label an object "coffee mug" once, "ceramic cup" the next time, and segment half the table as an object on the third pass. This inconsistency accumulates over time. Constraining the detector to an expected object list helps, but defeats the purpose of open-vocabulary detection.

These reflect a real gap between single-image detection — which current models handle well — and the persistent spatial inventory a robot actually needs.

![Spatial Memory Visualization](spatial-memory.jpg)

## Navigation

Autonomous navigation uses a ROS2 stack developed by the team at Dimensional, running in a Docker container on the Jetson. It fuses LiDAR scans with IMU data for SLAM, computes costmaps, plans paths, and outputs velocity commands.

Two WebSocket channels connect the nav stack to the workstation: one for visualization data (point cloud, pose, planned path) and one for velocity commands that the cmd_vel executor forwards to the robot's `Move()` function.

The nav stack runs in Docker because that's how Dimensional packaged it, and the ROS2 dependency tree is large enough that isolating it makes sense. Nothing else in the system uses ROS.

## Voice control

A voice interface connects the robot to OpenAI's Realtime API for speech-to-speech conversation. The robot listens through a USB microphone on the workstation (the Jetson's audio hardware is limited), and the LLM generates both spoken responses and tool calls.

Available tools include: navigate to a location, trigger a gesture, query spatial memory, and activate a manipulation skill. A routine executor can chain these into multi-step plans — "go to the kitchen, find the mug, pick it up" becomes a sequence of navigation goals, perception queries, and skill activations.

Single-step commands work reliably. Multi-step plans compound errors. If perception, navigation, and manipulation each succeed 85% of the time, a three-step chain succeeds 61% of the time. In practice, the rates are often lower. A navigation goal reached 30 cm off target means the subsequent grasp fails, and the planner doesn't know why.

The bottleneck is not the language model. Current LLMs plan well enough for this kind of task. The bottleneck is the state representation they receive. The planner can only act on what the perception system actually detected, which may not reflect what's in the room.

![Voice Interaction](voice-conversation.jpg)

## Teleoperation: the twelve-hour bug

Before the robot can learn manipulation, it needs training data. Before you can collect training data, you need working teleoperation. This turned out to be the hardest part of the project.

The setup: a Pico VR headset streams controller poses over WebSocket to the workstation. An IK solver converts 6-DoF wrist targets into 14-DoF joint angles. These are sent to the Jetson's arm relay, which publishes them to `rt/arm_sdk`.

The IK solver is Pink, a QP-based velocity IK library built on Pinocchio. It replaced a CasADi/IPOPT solver that was in the original codebase. Pink computes incremental velocity steps toward the target rather than re-solving the full nonlinear problem each frame. The difference: 0.5 ms per step vs 5-15 ms, and much less jitter.

A 0.7× workspace scaling maps human arm reach to the robot's shorter arms. Without it, extending your arm to 70% of your reach already pushes the robot to its kinematic limits.

After the IK solver, transforms, and scaling were all working correctly, the arms still couldn't lift above chest height. I could command the shoulder pitch to a position above the head. The arm would barely move. But pushing the arm by hand revealed it was compliant — it would hold a position if placed there, then slowly drift back. The motors had authority. Something was preventing them from using it.

I spent twelve hours on this. I replaced the IK solver. Added feed-forward gravity compensation torques. Tuned PD gains. Restructured the coordinate pipeline. Every change was correct. None had any visible effect.

The cause was a velocity clipper buried in the arm relay. The relay's control loop runs at 250 Hz. Each tick, the clipper read the motor's current position and capped how far the commanded position could deviate from it. The intent was smooth motion. The effect was that the PD controller never saw more than a tiny position error, so it never generated more than a fraction of available torque. The arm couldn't overcome gravity because the relay was preventing it from trying.

I removed the clipper. The arm lifted above the head on the first try.

## Learning manipulation

With teleoperation working, the pipeline is: collect demonstrations through VR, convert to training format, train a policy, deploy it.

I collected 50 episodes of a pick task using rubber grippers (the Dex3 hands had broken during earlier teleop sessions — no force feedback means you can't feel when fingers hit a surface). Each episode records the head camera feed and 29-DoF joint positions at 50 Hz, stored as operator-commanded targets.

The recordings convert to LeRobot format for training an ACT (Action Chunking with Transformers) policy. Given a camera image and current joint state, the policy predicts a chunk of future joint positions. 52 million parameters — ResNet18 vision backbone plus transformer decoder. Training on 50 episodes for 100k steps takes about 45 minutes on an RTX 5090. Loss drops from 7.7 to below 0.1.

Deployment uses the same arm relay as teleoperation. The skill runner moves the arms to a start pose, then runs inference at 50 Hz: read camera, read joints, predict action chunk, send targets. The policy runs until a termination condition or timeout.

The Dex3 hands deserve a mention. They're mechanically capable but not designed for this kind of abuse. Multiple fingers broke during data collection. When a thumb pin snapped, I measured it, modeled a replacement in CAD, and had a local CNC shop machine new ones. The hands are not designed to be disassembled.

## What the system looks like end to end

In the demo configuration, a user says "pick up the red mug." The voice agent queries spatial memory, retrieves the object's position, sends a navigation goal, waits for arrival, then activates the pick policy. Each step works individually. Chaining them reliably is the open problem.

This isn't a surprising result. It's the expected state of a system where every component is early-stage and the interfaces between them are thin. What the system demonstrates is the structure: what layers a humanoid robot OS needs, how they connect, and where they break down.

## Three things I'd change

**Standardize on containers from the start.** The microservice architecture happened because SAM3, FoundationPose, ROS2, and the voice model all need different Python environments. I solved the conflicts reactively with separate processes and Docker. Designing the communication interfaces first would have saved significant debugging.

**Build a proper spatial memory layer.** The current system stores object positions at detection time and never updates them. Objects move. The robot moves. A useful spatial inventory needs to track objects over time, merge observations, and handle uncertainty. This is a well-studied problem in robotics. Integrating it with foundation-model detectors is not.

**Interpolate between teleop commands.** The VR headset sends targets at 30 Hz. The arm relay runs at 250 Hz. Sending raw IK output directly to the motors produces visible vibration. A simple interpolation buffer would smooth this, improving both the teleoperation experience and the quality of collected training data.

---

*Pim Van den Bosch is a robotics engineer working on humanoid robot systems, simulation, and whole-body control.*
