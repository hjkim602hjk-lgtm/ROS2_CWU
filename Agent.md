---
title: 프로젝트 개발 원칙
aliases:
  - 개발 원칙
tags:
  - cwu/guidelines
---

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
* Intel RealSense depth camera — used for colour only (see Confirmed Camera Role)
* Four PSD distance sensors (model and interface not yet confirmed): one on the front-mounted gripper,
  plus one each on the left, right, and rear of the body
* Nucleo ↔ Pi link is moving from ST-Link USB to a direct UART (Nucleo PC10/PC11 = UART4 ↔ Pi GPIO15/14),
  because the USB link repeatedly froze under motor noise. Firmware build flags and ROS default port
  (`/dev/ttyAMA0`) are changed; upload, Pi UART setup, and on-robot verification are not done yet

## Confirmed Camera Role

* The camera does not measure depth. Use the colour image only.
* Its single job is to report which way the target lies, as a direction in the camera optical frame.
* Control keeps the target on the vertical centre line of the image: turn to null the horizontal
  error, and drive forward only once aligned.
* Distance is the gripper PSD's job, not the camera's. Do not reintroduce depth thresholds for
  approach, deceleration, or stopping.
* Do not publish a 3D target position. Without depth there is no distance to publish, and a pose
  message would invite consumers to treat an invented number as a coordinate.
* Camera mounting pitch is now a functional requirement, not a cosmetic one: if the target leaves
  the frame, the approach controller has no input at all. Measure it.
* Follow [[docs/superpowers/specs/2026-09-16-mission1-control-design|mission 1 control design]]
  for the interfaces and the state machine.

## Confirmed Gripper Behavior

* Mount the PSD sensor on the gripper. Automatically grasp when an object enters the configured detection distance.
* Only the gripper PSD may trigger grasping. The left/right/rear PSDs are for surrounding proximity sensing;
  their exact use, thresholds, and ROS interface are not decided yet.
* Use the same PSD sensor and gripper hardware in both missions.
* Keep target identification/approach and PSD proximity detection as separate responsibilities.
* Integration draft: enable automatic grasping only during target acquisition, stop the base before closing, and reject invalid or stale PSD readings.
* Proximity detection triggers grasping; it does not prove target identity or successful retention. Verify actual retention separately.
* Store calibrated detection distance and timing thresholds in YAML before the arena is revealed.
* Do not invent the PSD model, wiring, distance conversion, trigger distance, actuator limits, or MCU protocol. PSD/gripper control is not implemented yet.
* Follow [[docs/superpowers/specs/2026-09-16-psd-gripper-design|PSD automatic grasp design draft]] for the proposed integration.

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
* acquire it using PSD-triggered automatic grasping,
* transport it to its own starting location.

The system must not depend on a pre-built arena map.

## Mission 2 — Battle Rumble

The arena has no internal obstacles.

Four robots compete simultaneously.

The target is located near the center of the arena.

The robot should be capable of:

* detecting the target,
* acquiring the target using PSD-triggered automatic grasping and retaining it,
* detecting nearby robots,
* responding to physical interaction,
* preventing itself from entering elimination corner regions.

Pushing another robot is permitted, but damaging another robot is not.

## Software Architecture

Prefer modular ROS 2 nodes.

Recommended modules:

* sensor drivers
* LiDAR processing
* colour camera processing
* localization
* mapping
* navigation
* target detection
* target tracking
* gripper PSD input and target manipulation
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

## 역할 분담 — codex(설계) / Claude(검토)

이 프로젝트는 두 에이전트가 분업한다.

**codex = 설계자 (1차)**

* 요구사항을 받아 설계/구현 초안을 작성한다.
* 산출물은 `docs/superpowers/plans/` 또는 `docs/superpowers/specs/` 의 문서, 혹은 실제 코드 변경.
* 작업 후 반드시 남길 것: 변경한 파일 목록, 설계 의도 한 줄, 확신이 없는 부분(열린 질문).
* 스스로 자기 설계를 승인하지 않는다. 검토 전에는 "초안" 상태다.

**Claude = 검토자 (2차)**

* codex 산출물을 받아 검사하고 **직접 수정**한다. 지적만 하고 넘기지 않는다.
* 검토 순서:
  1. `docs/competition_rules.md` 위반 여부 — 특히 온보드 연산, 미션 중 외부 통신, 아레나 공개 후 파라미터 수정 금지.
  2. 실제 동작 여부 — ROS 2 토픽/프레임/QoS 연결, 런치 인자, 빌드.
  3. 하드웨어 현실성 — Pi 4 4GB 연산량, G4 스펙 범위, 질량/치수 제약.
  4. 과설계 — 안 쓰는 추상화, 한 번만 쓰이는 인터페이스, 죽은 설정값은 삭제.
  5. 검증 — `colcon build` + 해당 패키지 테스트. 하드웨어 없이 돌 수 있어야 한다.
* 보고 형식: `수정함: ... / 남긴 이유: ... / codex가 답해야 할 것: ...`
* 검토를 통과하지 못한 설계는 "통과했다"고 말하지 않는다. 실행 결과를 근거로 말한다.

**경계**

* codex가 이미 정한 설계를 취향 문제로 다시 쓰지 않는다. 규칙 위반, 동작 불가, 제약 초과, 과설계 — 이 네 가지만 수정 사유다.
* 근거가 규칙 문서에 있으면 해당 조항을 인용한다.

## 관련 문서

- [[docs/프로젝트 목차|프로젝트 목차]] — 전체 문서와 파일 탐색
- [[docs/competition_rules|개발 제약의 근거]]
- [[README|빌드 및 실행 절차]]
