# Autonomous Robot Competition Project

## Project Goal

Develop an autonomous mobile robot for the 2026 Changwon National University Autonomous Robot Competition.

The robot must perform all perception, localization, navigation, planning, manipulation, and decision-making onboard.

The competition rules in `docs/competition_rules.md` are the source of truth for competition requirements.

## Development Environment

* Ubuntu 22.04
* ROS 2 Humble
* Raspberry Pi 4 4GB as the onboard computer
* Python is preferred for ROS 2 nodes unless performance requires C++
* YDLIDAR G4
* Intel RealSense depth camera

## Mandatory Competition Constraints

The following constraints must never be violated.

1. All autonomous software must run on the onboard computer.

2. Do not design any system that requires remote computation, cloud inference, remote SLAM, remote visualization, or external PC control during a mission.

3. After a mission starts, the robot must operate without external communication.

4. The external PC may only be treated as a mechanism for issuing the permitted mission execution command before autonomous operation.

5. Do not create an architecture that requires modifying source code, configuration files, maps, parameters, or calibration values after the arena is revealed.

6. The robot must autonomously adapt to the arena using onboard sensors.

7. Hardware must remain unchanged between Mission 1 and Mission 2.

8. Mission-specific software and launch configurations may be separate.

9. Robot design assumptions must respect:

   * Maximum mass: 10 kg
   * Maximum expanded envelope: 400 mm diameter × 300 mm height

10. Do not implement mechanisms intended to damage another robot, interfere with another robot's sensors, or damage the arena.

## Mission 1 — Autonomous Search and Rescue

The arena is 3600 mm × 3600 mm and divided into a 9 × 9 grid.

Obstacles may occupy grid cells.

The target is approximately:

* Red
* 50 mm × 50 mm × 50 mm
* approximately 100 g
* initially located at grid position (5,5)

The robot must autonomously:

* explore the arena,
* avoid obstacles,
* locate the target,
* approach the target,
* acquire it,
* transport it to its own starting location.

The system must not depend on a pre-built arena map.

## Mission 2 — Battle Rumble

The arena has no internal obstacles.

Four robots compete simultaneously.

The target is located near the center of the arena.

The robot should be capable of:

* detecting the target,
* acquiring and retaining the target,
* detecting nearby robots,
* responding to physical interaction,
* preventing itself from entering elimination corner regions.

Pushing another robot is permitted, but damaging another robot is not.

## Software Architecture

Prefer modular ROS 2 nodes.

Recommended modules:

* sensor drivers
* LiDAR processing
* depth camera processing
* localization
* mapping
* navigation
* target detection
* target tracking
* target manipulation
* opponent detection
* mission state machine
* safety supervisor

Do not create large monolithic ROS 2 nodes unless there is a clear technical reason.

## Development Rules

Before changing code:

1. Inspect the existing project.
2. Identify which ROS 2 nodes and interfaces are affected.
3. Explain the proposed change briefly.
4. Implement the smallest reasonable change.
5. Build and test the affected package.

Use ROS 2 standard interfaces where practical.

Avoid unnecessary dependencies.

Parameters that require tuning must be stored in YAML configuration files instead of being hard-coded.

However, the final competition system must not require participant modification of those parameters after the arena is revealed.

## Validation

Whenever relevant, run:

* `colcon build`
* ROS 2 node/interface checks
* unit tests
* static checks
Never assume hardware is available when running automated tests.

Separate hardware-independent logic so it can be tested without the physical robot.
