# 로봇 본체(base_link) 기준 센서 장착값을 읽어 정적 TF 노드로 만드는 공용 모듈입니다.
# LiDAR와 D415가 같은 mount.yaml을 쓰도록 하여 TF 트리 정의를 한 곳에 둡니다.
# 값 검사와 미보정 경고도 여기서 한 번만 합니다.
#
# [공부 노트] 장착값(mount)이 뭔가?
#   "로봇 중심(base_link)에서 LiDAR 가 앞으로 몇 m, 왼쪽으로 몇 m, 위로 몇 m 에 있고,
#    얼마나 돌아가 붙어 있나" 라는 6개 숫자입니다.
#     x : 앞(+) / 뒤(-)      y : 왼쪽(+) / 오른쪽(-)      z : 위(+)        [m]
#     roll : x축 기울기      pitch : y축 기울기(고개 숙임)  yaw : 좌우로 돌린 각도   [라디안]
#   이 값이 틀리면 LiDAR 가 본 벽이 지도에서 엉뚱한 곳에 찍혀 지도가 번집니다.
#   값은 config/mount.yaml 에 적고, 이 파일이 읽어서 static TF(base_link → laser_frame)로 발행합니다.
#
# [공부 노트] static TF 란?
#   로봇이 움직여도 변하지 않는 좌표 관계(나사로 고정된 센서 위치)는 한 번만 발행하면 됩니다.
#   ROS 기본 제공 프로그램 tf2_ros/static_transform_publisher 가 그 일을 합니다.
#   이 파일은 그 프로그램을 "어떤 인자로 띄울지" launch 용 Node 객체를 만들어 줄 뿐입니다.

"""Shared loader for base_link → sensor static transforms."""
import math  # isfinite: 숫자가 inf/NaN 이 아닌지 검사

from launch_ros.actions import Node  # launch 에서 띄울 노드 하나를 표현하는 객체
import yaml                          # YAML 파일 읽기 (apt: python3-yaml)

# 장착값 6개의 이름. 이 순서대로 검사하고 인자를 만듭니다.
KEYS = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')


# ─────────────────────────────────────────────────────────────────────────────
# load_mount: YAML 에서 센서 하나의 장착값 6개를 읽어 검사한 뒤 사전으로 돌려줍니다.
#   필요한 입력
#     path : mount.yaml 경로
#     name : 그 안의 항목 이름 (예: 'laser_mount')
#   돌려주는 값: {'x': 0.05, 'y': 0.0, ..., 'yaw': 0.0}  (모두 float)
#   값이 빠졌거나 숫자가 아니면 ValueError → launch 가 시작 단계에서 멈춤 (틀린 채로 달리지 않게)
# ─────────────────────────────────────────────────────────────────────────────
def load_mount(path, name):
    """Return the six finite mount values of `name` from the YAML at `path`."""
    # with open(...) as handle: 파일을 열고, 블록이 끝나면 자동으로 닫아줌
    with open(path, encoding='utf-8') as handle:
        # safe_load: YAML → 파이썬 사전/리스트. (safe = 임의 코드 실행 위험 없는 버전)
        data = yaml.safe_load(handle)
    # 파일이 비었거나(None) 사전이 아니거나 해당 항목이 없으면 에러
    if not isinstance(data, dict) or name not in data:
        raise ValueError("%s에 '%s' 항목이 없습니다" % (path, name))
    # 항목은 있는데 값이 비어 있으면(None) 빈 사전으로 취급 → 아래에서 "값 없음" 에러가 남
    mount = data[name] or {}
    result = {}
    for key in KEYS:
        if key not in mount:
            raise ValueError("%s의 '%s'에 %s 값이 없습니다" % (path, name, key))
        value = mount[key]
        # true/false 는 파이썬에서 1/0 으로도 취급되므로 따로 막음. inf/NaN 도 막음.
        if isinstance(value, bool) or not math.isfinite(float(value)):
            raise ValueError('Mount values must be finite numbers')
        # 정수(0)로 적었어도 실수(0.0)로 통일
        result[key] = float(value)
    return result


# ─────────────────────────────────────────────────────────────────────────────
# warn_if_uncalibrated: 6개 값이 전부 0 이면 "아직 안 쟀다"로 보고 경고만 찍습니다.
#   (에러로 멈추지는 않음 — 데모/시험에서는 0 으로 돌려볼 수 있어야 하므로)
#   필요한 입력: load_mount 결과, 로그에 쓸 이름(label)
# ─────────────────────────────────────────────────────────────────────────────
def warn_if_uncalibrated(mount, label):
    """Print a warning when every value is zero, which means "not measured yet"."""
    # 원점 장착 자체는 가능하지만 실물에서 여섯 값이 전부 0이면 미측정이라는 뜻입니다.
    # 그대로 두면 SLAM·인식은 계속 돌면서 조용히 어긋난 결과를 냅니다.
    # any(...) : 하나라도 0 이 아니면 True → 잰 값이 있다고 보고 조용히 끝
    if any(mount[key] for key in KEYS):
        return
    # launch 단계라 ROS 로거가 없어서 print 로 터미널에 출력
    print('[cwu_slam] 경고: %s 장착값이 전부 0입니다 (미보정 기본값). '
          'base_link 기준 위치·각도를 측정해 입력하십시오.' % label)


# ─────────────────────────────────────────────────────────────────────────────
# static_tf_node: 장착값으로 static_transform_publisher 노드를 만들어 돌려줍니다.
#   필요한 입력
#     name         : 노드 이름 (예: 'laser_mount')
#     child_frame  : 센서 좌표계 이름 (예: 'laser_frame' — LiDAR 드라이버의 frame_id 와 같아야 함!)
#     mount        : load_mount 결과
#     use_sim_time : 가짜 시계 사용 여부
#     parent_frame : 부모 좌표계, 기본 'base_link'
#   결과를 launch 목록에 넣으면 실행 시 아래 명령과 같은 일이 일어납니다:
#     ros2 run tf2_ros static_transform_publisher --x 0.05 ... \
#         --frame-id base_link --child-frame-id laser_frame
# ─────────────────────────────────────────────────────────────────────────────
def static_tf_node(name, child_frame, mount, use_sim_time, parent_frame='base_link'):
    """Build the tf2_ros static publisher for one measured sensor mount."""
    args = []
    # ['--x', '0.05', '--y', '0.0', ...] 형태의 명령줄 인자 만들기 (인자는 전부 문자열)
    for key in KEYS:
        args.extend(['--' + key, str(mount[key])])
    return Node(
        package='tf2_ros', executable='static_transform_publisher', name=name,
        arguments=args + ['--frame-id', parent_frame, '--child-frame-id', child_frame],
        parameters=[{'use_sim_time': use_sim_time}], output='screen')
