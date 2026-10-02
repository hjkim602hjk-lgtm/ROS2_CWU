# 엔코더 틱을 로봇 위치로 바꾸는 순수 계산 함수 모음입니다.
# 카운터 되감김 보정, 틱 → 이동거리 환산, 차동구동 호(arc) 적분을 담당합니다.
# ROS와 하드웨어에 의존하지 않으므로 로봇 없이 pytest로 검증합니다.
#
# [공부 노트] 이 파일이 전체 흐름에서 하는 일
#   Nucleo ──"ENC,1499,1406"──> encoder_odom.py ──(틱 숫자)──> 이 파일 ──(x, y, theta)──> /odom
#   - 이 파일은 "숫자 계산"만 합니다. 시리얼·ROS 토픽은 전혀 모릅니다.
#   - 그래서 로봇 없이도 pytest로 "틱 3009.5개 = 바퀴 한 바퀴 = 0.26 m" 같은 걸 시험할 수 있습니다.
#   - "계산(순수 함수)"과 "입출력(노드)"을 파일로 나누는 게 이 프로젝트의 기본 패턴입니다.
#     (detect.py ↔ target_detector.py, servo.py ↔ target_follower.py 도 같은 구조)
#
# [공부 노트] 차동구동(differential drive)이란?
#   왼쪽·오른쪽 바퀴 두 개를 각각 돌려서 움직이는 로봇입니다.
#   - 두 바퀴가 같은 거리 → 직진
#   - 오른쪽이 더 많이 → 왼쪽으로 휨(반시계, theta 증가)
#   - 서로 반대 방향 같은 거리 → 제자리 회전

"""Pure differential-drive odometry maths, testable without ROS or hardware."""
# math: 파이썬 표준 라이브러리. pi, sin, cos, atan2 를 씁니다. 설치 필요 없음.
import math


# ─────────────────────────────────────────────────────────────────────────────
# wrap_delta: "이전 틱"과 "지금 틱"의 차이(= 이번에 굴러간 틱)를 구합니다.
#   필요한 입력
#     previous     : 지난번에 받은 누적 틱 (정수)
#     current      : 이번에 받은 누적 틱 (정수)
#     counter_bits : Nucleo 카운터의 비트 수. STM32의 long 은 32비트 → encoder.yaml 에 32
#   돌려주는 값: 이번에 움직인 틱 수 (앞으로 +, 뒤로 -)
# ─────────────────────────────────────────────────────────────────────────────
def wrap_delta(previous, current, counter_bits):
    """누적 카운터가 되감겨도 올바른 증분을 돌려줍니다.

    MCU의 틱 카운터는 보통 고정 비트 폭의 부호 있는/없는 정수라 최대값에서
    최소값으로 점프합니다. 그 점프를 실제 이동으로 착각하면 지도가 한순간에
    튀므로, 증분이 카운터 범위의 절반을 넘으면 되감김으로 보고 보정합니다.
    counter_bits가 0이면 되감김이 없는 것으로 보고 그대로 뺍니다.
    """
    # 기본: 지금 값 - 이전 값 = 이번에 움직인 양
    delta = current - previous
    # 비트 수가 0 이하 = "카운터가 넘칠 일 없음"으로 설정한 경우 → 그냥 뺀 값 사용
    if counter_bits <= 0:
        return delta
    # span = 카운터가 표현할 수 있는 값의 개수. 32비트면 2^32 = 약 43억.
    # (1 << n) 은 1을 왼쪽으로 n칸 민다 = 2의 n제곱. 정수 연산이라 빠르고 정확합니다.
    span = 1 << counter_bits
    # 절반을 넘는 증분은 반대 방향의 작은 증분으로 해석하는 쪽이 항상 옳습니다.
    # 예) 8비트(span=256) 카운터가 250 → 4 로 바뀌었다면 단순 차이는 -246 이지만
    #     실제로는 앞으로 10틱 간 것(250→255→0→4). 아래 식이 이걸 +10 으로 바꿔 줍니다.
    #     (delta + 128) % 256 - 128  →  결과를 항상 -128 ~ +127 범위로 접는 공식
    return (delta + span // 2) % span - span // 2


# ─────────────────────────────────────────────────────────────────────────────
# ticks_to_metres: 틱 수 → 바닥 위를 굴러간 거리(m)
#   필요한 입력 (둘 다 encoder.yaml 의 실측값)
#     ticks_per_rev : 바퀴가 딱 한 바퀴 돌 때 틱 수 (3009.5)
#     wheel_radius  : 바퀴 반지름 m (0.04151)
#   원리: (몇 바퀴 돌았나) × (바퀴 둘레 2πr)
# ─────────────────────────────────────────────────────────────────────────────
def ticks_to_metres(ticks, ticks_per_rev, wheel_radius):
    """바퀴 틱 수를 바닥 위 이동 거리(m)로 바꿉니다."""
    # ticks / ticks_per_rev       → 바퀴가 몇 바퀴 돌았는지 (예: 1504.75 / 3009.5 = 0.5바퀴)
    # 2 * math.pi * wheel_radius  → 바퀴 한 바퀴 둘레 (약 0.261 m)
    return ticks / ticks_per_rev * (2 * math.pi * wheel_radius)


# ─────────────────────────────────────────────────────────────────────────────
# differential_step: 짧은 한 순간(약 0.1초) 동안 좌·우 바퀴가 간 거리로
#                    "로봇 자신 기준으로" 얼마나 움직였는지 계산합니다.
#   필요한 입력
#     left, right      : 좌·우 바퀴 이동 거리 (m) ← ticks_to_metres 결과
#     wheel_separation : 좌우 바퀴 사이 거리(트레드, m) = 0.20686 (encoder.yaml)
#   돌려주는 값: (앞으로 간 거리 dx, 옆으로 밀린 거리 dy, 회전한 각도 dtheta[라디안])
#   ※ "로봇 기준"이라 dx 는 로봇 코가 향한 방향, dy 는 로봇 왼쪽 방향입니다.
# ─────────────────────────────────────────────────────────────────────────────
def differential_step(left, right, wheel_separation):
    """좌우 바퀴 이동거리로부터 로봇 기준(body frame) 변위를 구합니다.

    직선 근사가 아니라 호를 그대로 적분합니다. 제자리 회전과 급선회에서
    직선 근사는 오차가 빠르게 누적되기 때문입니다.
    """
    # 로봇 중심이 간 거리 = 두 바퀴 거리의 평균
    distance = (left + right) / 2.0
    # 회전각(라디안) = 두 바퀴 거리 차이 ÷ 바퀴 간격
    #   오른쪽이 더 가면 양수 = 왼쪽(반시계)으로 돎. ROS 규칙(반시계가 +)과 같습니다.
    #   wheel_separation 이 틀리면 여기가 틀려서 "360도 돌았는데 354도로 계산"되는 문제가 생깁니다.
    heading = (right - left) / wheel_separation
    # 거의 안 돌았으면(직진) 아래에서 0으로 나누게 되므로 따로 처리:
    # 그냥 앞으로 distance 만큼, 옆으로 0.
    if abs(heading) < 1e-9:
        return distance, 0.0, heading
    # 휘면서 갔으면 로봇은 원호(arc) 위를 움직인 것. 그 원의 반지름 = 호 길이 ÷ 각도
    radius = distance / heading
    # 원호 끝점 좌표 공식: 앞으로 r·sin(θ), 옆으로 r·(1-cos(θ))
    return radius * math.sin(heading), radius * (1 - math.cos(heading)), heading


# ─────────────────────────────────────────────────────────────────────────────
# integrate: 지금까지의 위치(pose)에 이번 한 스텝을 더해 새 위치를 만듭니다.
#   필요한 입력
#     pose  : (x, y, theta) — odom 좌표계(= 전원 켠 자리 기준) 위치와 방향
#     left, right, wheel_separation : differential_step 과 같음
#   핵심: differential_step 결과는 "로봇 기준"이라, 로봇이 지금 향한 방향(theta)만큼
#        돌려서(회전 변환) "출발점 기준" 좌표로 바꾼 뒤 더해야 합니다.
# ─────────────────────────────────────────────────────────────────────────────
def integrate(pose, left, right, wheel_separation):
    """이전 자세 (x, y, theta)에 한 스텝을 더한 새 자세를 돌려줍니다."""
    # 튜플 풀기: pose = (1.0, 2.0, 0.5) 이면 x=1.0, y=2.0, theta=0.5
    x, y, theta = pose
    # 로봇 기준 이동량
    dx, dy, dtheta = differential_step(left, right, wheel_separation)
    # 2D 회전 행렬 [cos -sin; sin cos] 로 "로봇 기준 → 출발점 기준" 변환 후 더하기.
    # 예) 로봇이 90도(왼쪽)를 보고 있을 때 "앞으로 1 m"는 출발점 기준으로 y+1 m 입니다.
    return (x + dx * math.cos(theta) - dy * math.sin(theta),
            y + dx * math.sin(theta) + dy * math.cos(theta),
            normalise(theta + dtheta))


# ─────────────────────────────────────────────────────────────────────────────
# normalise: 각도를 -π ~ +π (-180도 ~ +180도) 범위로 맞춥니다.
#   왜? 계속 돌면 각도가 7.0, 13.5 … 처럼 커지는데, 370도와 10도는 같은 방향입니다.
#   범위를 하나로 통일해야 두 각도를 비교하거나 뺄 때 헷갈리지 않습니다.
# ─────────────────────────────────────────────────────────────────────────────
def normalise(angle):
    """각도를 -pi ~ pi 범위로 접습니다."""
    # sin/cos 로 바꿨다가 atan2 로 되돌리면 자동으로 -π~π 범위의 같은 방향 각도가 나옵니다.
    return math.atan2(math.sin(angle), math.cos(angle))
