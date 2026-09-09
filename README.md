# CWU LiDAR 2D SLAM

ROS 2 Humble / Ubuntu 22.04 / YDLIDAR G4 / 엔코더 기반 이동 로봇용 구성입니다.
SLAM Toolbox가 지도와 위치를 추정합니다. 라즈베리파이 4에서는 센서 드라이버,
엔코더 오도메트리와 SLAM을 모두 온보드로 실행합니다.

현재 포함된 기능:

- G4 드라이버 실행, 센서 장착 TF, SLAM Toolbox 설정
- 장비 없이 실행하는 **별도 가상 센서 데모**와 지도 생성·저장 통합 검증
- 선택적인 개발용 RViz 화면

**아직 실물 검증 전입니다.** 엔코더가 있다는 조건만 확인되었으므로 엔코더
통신/틱 변환 노드는 포함하지 않습니다. 해당 드라이버가 아래 `/odom`과 TF를
제공해야 실제 이동 중 지도 생성이 가능합니다. 모터 제어·자율 탐색·Nav2 주행은
이 패키지의 범위에 포함하지 않습니다.

## 1. 빌드

현재 PC에는 SLAM/지도 저장 의존성을 설치했습니다. 새 PC 또는 라즈베리파이의
Ubuntu 22.04 64-bit + ROS 2 Humble 환경에서는 다음을 실행합니다.

```bash
sudo apt update
sudo apt install ros-humble-slam-toolbox ros-humble-nav2-map-server \
  ros-humble-rviz2 python3-colcon-common-extensions python3-rosdep \
  python3-pytest python3-yaml build-essential cmake
cd ~/ros_CWU
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
MAKEFLAGS=-j2 colcon build --symlink-install --executor sequential
source install/setup.bash
```

`rosdep`을 처음 쓰는 시스템은 `sudo rosdep init`과 `rosdep update`를 먼저 합니다.
ARM 라즈베리파이로 옮길 때 `build/`, `install/`, `log/`는 복사하지 않고 다시 빌드합니다.
`colcon.meta`가 SDK → G4 드라이버 순서를 지정하며 SDK도 작업공간 내부에 설치합니다.
SDK의 빈 라이브러리 경로 내보내기 문제를 해결하는 작은 드라이버 CMake 패치를
적용했습니다(`patches/ydlidar-workspace-library.patch`). 제조사 자체 테스트는 빌드에서 제외했습니다.
자체 SLAM 패키지 테스트는 아래에서 별도로 실행합니다.

하드웨어 드라이버는 선택 의존성이므로 `--packages-up-to cwu_slam`만으로는 빌드되지
않습니다. 위 전체 빌드 명령을 사용하세요. 데모만 필요하면 다음으로 충분합니다.

```bash
colcon build --symlink-install --packages-select cwu_slam
```

제조사 소스가 없는 새 체크아웃에서는 작업공간 루트에서 `dependencies.repos`의
커밋을 사용하거나 다음처럼 가져옵니다.

```bash
git clone https://github.com/YDLIDAR/YDLidar-SDK.git src/ydlidar_sdk
git -C src/ydlidar_sdk checkout 01cdda4f2b36dff2a706d0535c64228d863c7411
git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git src/ydlidar_ros2_driver
git -C src/ydlidar_ros2_driver checkout 4ef70d3f32a85704ade0be54b214f3763b1ab3e8
git -C src/ydlidar_ros2_driver apply ../../patches/ydlidar-workspace-library.patch
```

## 2. 장비 없는 데모

```bash
cd ~/ros_CWU
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch cwu_slam demo.launch.py rviz:=true
```

GUI가 없으면 `rviz:=true`를 생략합니다. 가상 로봇이 원을 돌며 3.6 m 방과 내부
상자에 대한 `/scan`, `/odom`, TF를 발행합니다. SLAM이 **스캔으로부터** 지도를 만듭니다.
데모 방은 테스트용이며 실물 실행에 로드되지 않습니다. 가상 입력은 이상적인
오도메트리와 순간 스캔으로, 슬립·노이즈·스캔 중 움직임을 재현하지 않습니다.
따라서 데모 통과는 실물 정확도나 라즈베리파이 성능을 보증하지 않습니다.

데모는 기본적으로 실제 시계를 사용합니다. 외부 `/clock` 발행기 없이
`use_sim_time:=true`를 설정하면 타이머가 동작하지 않습니다. 종료는 Ctrl+C입니다.

## 3. 실제 G4 + 엔코더 연결

```text
G4 driver ── /scan ────────────────────┐
Encoder driver ── odom → base_link TF ─┼─ SLAM Toolbox ── /map
Mount TF ── base_link → laser_frame ───┘                 map → odom TF
```

TF별 발행자는 반드시 하나여야 합니다.

| 인터페이스 | 제공자 | 조건 |
|---|---|---|
| `/scan` (`sensor_msgs/LaserScan`) | G4 드라이버 | `frame_id=laser_frame`, 실제 측정 타임스탬프 |
| `/odom` (`nav_msgs/Odometry`) | 엔코더 드라이버 | `header.frame_id=odom`, `child_frame_id=base_link` |
| `odom → base_link` | 엔코더 드라이버 또는 robot_localization | 스캔 시각에 조회 가능한 동적 TF |
| `base_link → laser_frame` | 이 패키지 또는 robot_state_publisher | 측정한 센서 장착 위치·방향 |
| `map → odom` | SLAM Toolbox | 다른 노드에서 중복 발행하지 않음 |

**SLAM Toolbox는 `/odom` 토픽 자체가 아니라 TF를 사용합니다.** `/odom`만 발행해도
SLAM 입력 조건이 충족되는 것은 아닙니다. 이동 로봇에 고정 `odom → base_link` TF를
넣어 문제를 숨기면 안 됩니다. 보통 엔코더 오도메트리를 20–50 Hz 정도로 발행하고,
LiDAR와 동일한 ROS 시계의 측정 시각을 사용합니다.

실물 투입 전에 다음 하드웨어 값을 확정합니다. 경기장 공개 뒤 조정하는 값이 아닙니다.

- `src/cwu_slam/config/mount.yaml`: 현재 원점/무회전인 **미보정 기본값**입니다.
  실제 장착 x/y/z(미터), roll/pitch/yaw(라디안)를 측정합니다.
  REP-103 기준 +x 전방, +y 좌측, +z 위쪽입니다.
- `ydlidar_g4.yaml`: 제조사 G4 설정을 바탕으로 230400 baud, 9 kHz, 10 Hz를 사용합니다.
  실물에서 포인트 방향과 반전 설정을 확인합니다. 직렬 장치는 가능하면
  `/dev/serial/by-id/...` 고정 경로를 사용합니다.
- 엔코더 드라이버: 구동 방식, 바퀴 반지름/간격, 감속비 반영 후 회전당 틱 수,
  좌우 부호, 누적 카운터 overflow/reset, 시간 단위, 공분산과 통신 끊김 처리가 필요합니다.
  차동구동·메카넘 등을 아직 가정하지 않았습니다.

엔코더/로봇 드라이버를 먼저 실행한 뒤:

```bash
source /opt/ros/humble/setup.bash
source ~/ros_CWU/install/setup.bash
ros2 launch cwu_slam slam.launch.py port:=/dev/ttyUSB0
```

URDF가 센서 TF를 이미 발행하면 `publish_mount_tf:=false`를 추가합니다.
G4 드라이버도 별도로 실행 중이면 `start_lidar:=false`를 추가합니다.
이 실물 launch에는 가상 데이터나 모터 명령이 없습니다. 로봇이 실제로 이동해야
새 영역을 지도에 넣을 수 있습니다.

직렬 접근이 거부되면 현재 사용자의 `dialout` 그룹 권한을 확인합니다. 필요한 경우
`sudo usermod -aG dialout "$USER"` 실행 후 로그아웃/로그인합니다.
이 작업에서는 사용자 그룹이나 USB 규칙을 변경하지 않았습니다.

## 4. 지도 저장 및 입력 확인

SLAM이 실행 중인 동안 같은 ROS 환경을 source한 다른 터미널에서:

```bash
mkdir -p ~/ros_CWU/maps
ros2 run nav2_map_server map_saver_cli -f ~/ros_CWU/maps/current \
  --ros-args -p save_map_timeout:=10.0
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map laser_frame
```

`current.pgm`과 `current.yaml`은 점유 지도의 결과물이며 SLAM 포즈 그래프는 아닙니다.
실제 SLAM은 매 실행마다 새 지도를 만들며 이 파일을 자동 로드하지 않습니다.
스캔이 보여도 지도가 없으면 TF의 존재·시각·프레임 이름부터 확인합니다.

실제 기록 데이터가 있다면 가상 센서 없이 재생할 수 있습니다.

```bash
ros2 launch cwu_slam slam.launch.py start_lidar:=false \
  publish_mount_tf:=false use_sim_time:=true
# 다른 터미널: /scan, /tf, /tf_static이 포함된 기록이어야 합니다.
ros2 bag play <bag_directory> --clock
```

bag에 센서 장착 TF가 없으면 올바른 `mount_file`로 `publish_mount_tf:=true`를 사용합니다.

## 5. 테스트

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select cwu_slam --event-handlers console_direct+
colcon test-result --verbose
# 별도 데모 실행 없이 시작/검증/저장/종료까지 자동 수행
python3 src/cwu_slam/test/run_demo_check.py
```

통합 검증은 기본 ROS 도메인 67과 localhost 통신을 사용하며 `/tmp/cwu-slam-check-*`에
지도와 로그를 남깁니다. 같은 도메인에 다른 로봇/데모를 실행하지 마세요.
`ROS_DOMAIN_ID=68 python3 ...`처럼 빈 도메인을 지정할 수 있습니다.
소켓 통신이 차단된 샌드박스에서는 ROS 통합 테스트에 별도 실행 권한이 필요합니다.

기본 테스트는 레이 교차, 가까운 장애물 차폐, 평행/후방 선분, 범위 제한을 검사합니다.
통합 검증은 움직이는 odom, 유효한 스캔, 전체 TF 연결, 지도 셀과 저장 파일을 확인합니다.
실물 G4/엔코더, 실제 루프 폐쇄 품질, Pi 메모리·CPU 부하는 별도 실험이 필요합니다.

경기에서는 RViz/외부 PC에 의존하지 않고 모든 계산을 온보드로 실행해야 합니다.
규칙은 현재 `Agent.md`를 기준으로 했으며 그 문서에서 참조하는
`docs/competition_rules.md`는 아직 작업공간에 없습니다.

## 참고

주석을 직접 넣기 어려운 보조 파일의 역할:

- `colcon.meta`: JSON 형식의 빌드 설정입니다. SDK를 G4 드라이버보다 먼저
  빌드하도록 지정하고 제조사 테스트·예제 빌드를 끕니다. JSON은 주석을 지원하지 않습니다.
- `src/cwu_slam/resource/cwu_slam`: ament가 ROS 패키지를 찾을 때 사용하는
  빈 등록 파일입니다. 실행 코드나 설정값을 담는 파일이 아닙니다.
- `patches/ydlidar-workspace-library.patch`: 제조사 드라이버가 작업공간에 설치된
  SDK 라이브러리를 찾도록 CMake를 수정하는 재현용 패치입니다.

- [SLAM Toolbox Humble](https://docs.ros.org/en/humble/p/slam_toolbox/)
- [G4 ROS 2 드라이버](https://github.com/YDLIDAR/ydlidar_ros2_driver/tree/humble)
- [YDLidar SDK](https://github.com/YDLIDAR/YDLidar-SDK)
