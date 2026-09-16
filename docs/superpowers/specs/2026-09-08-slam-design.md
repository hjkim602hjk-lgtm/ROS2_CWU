---
title: 엔코더 기반 2D SLAM 설계
aliases:
  - SLAM 설계
tags:
  - cwu/design
---

# Encoder-assisted 2D SLAM

User confirmed wheel encoders after the proposed Humble/G4/SLAM Toolbox design.
Target: Ubuntu 22.04, ROS 2 Humble, Raspberry Pi 4 4GB. All mission computation
runs onboard; no prebuilt arena map or external communication is required.

Create `cwu_slam`, an ament_python bringup package. SLAM Toolbox consumes `/scan`
and the encoder driver's timestamped `odom -> base_link` TF. The encoder driver
should also publish `nav_msgs/Odometry` on `/odom`; SLAM Toolbox uses TF, not that
topic. Do not invent wheel geometry, tick encoding, robot kinematics, or transport.
The driver implementation is outside this package until those are specified.

The package owns `map -> odom` via SLAM Toolbox and optionally a static
`base_link -> laser_frame` mount transform. Real bringup starts the G4 driver;
an existing URDF can instead own the mount transform. Hardware calibration is
required before deployment, never arena-specific post-reveal tuning.

A separate demo launch supplies synthetic scans and exact odometry on a closed
trajectory in an asymmetric room. It uses wall time, not a nonexistent /clock.
The synthetic map is only a test fixture and never loaded in real mode.
The demo has no motor command publisher. Parameters live in YAML. RViz is optional
and off by default. Map saving is explicit through Nav2 map_saver_cli.

Validation: ray/segment numerical unit tests; colcon build/test; launch the real
SLAM backend against synthetic inputs, verify free and occupied map cells and
the TF chain, and save/reload a map artifact. This proves integration, not encoder
accuracy, G4 hardware operation, loop closure quality, or Raspberry Pi throughput.

## 설계 이후 추가된 것 (문서 미반영)

이 설계에 없지만 코드에는 들어간 것들입니다. 사실만 적었고 채택 근거는 설계자가
채워야 합니다.

- `cwu_base` 패키지: 위에서 "이 패키지 밖"이라고 미뤄 둔 엔코더 드라이버가 실제로
  구현됐습니다. 시리얼 틱 → `/odom` + `odom → base_link` TF. 프로토콜은 미확정이라
  `encoder.yaml`의 필드 번호·바퀴 제원을 실측해 채우는 방식입니다.
- Cartographer 백엔드: `backend:=cartographer`로 SLAM Toolbox 대신 고를 수 있습니다
  (`config/cartographer_2d.lua`). 설계 본문은 SLAM Toolbox 단일 백엔드를 전제합니다.
- 웹 UI: `web.launch.py`(rosbridge + 정적 HTML). 설계의 "RViz는 선택이고 기본 꺼짐"
  범위 밖입니다. 미션 중 외부 통신 금지 규정 때문에 **개발용 전용**입니다.

> [!info] 작성 당시의 기준
> 이 설계 작성 시에는 Agent.md만 참고했습니다. 현재는 [[docs/competition_rules|대회 규정 정리]]가 추가되어 있습니다.

## 관련 문서

- [[docs/프로젝트 목차|프로젝트 목차]] — 전체 문서와 파일 탐색
- [[docs/competition_rules|현재 확보된 규정 정리]]
- [[docs/superpowers/plans/2026-09-08-slam|설계를 구현한 계획과 검증 기록]]
- [[README|현재 실행 방법과 하드웨어 상태]]
