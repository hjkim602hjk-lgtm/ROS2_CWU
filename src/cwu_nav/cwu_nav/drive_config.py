# [공부 노트] 이 파일이 하는 일
#   motor.yaml 의 실측값(로봇 반경, 최고 속도, 제동거리 …) 한 벌을 nav2.yaml 의 여러 곳에 똑같이 써 넣습니다.
#   같은 숫자를 사람이 여러 파일에 손으로 옮기면 언젠가 하나가 어긋나기 때문입니다.
#   nav2.launch.py 가 실행할 때마다 불러서, 결과를 임시 YAML 로 저장해 Nav2 에 넘깁니다.
#   (원본 nav2.yaml 파일은 바꾸지 않음 → 경기장 공개 후 파일 수정 금지 규칙과도 맞음)
#
#   필요한 입력
#     config    : nav2.yaml 을 읽은 사전 (이 함수가 직접 고쳐서 돌려줌)
#     directory : config 폴더 경로 (행동 트리 XML 위치)
#     measured  : motor.yaml 의 motor_bridge 파라미터 사전. None = 실물 구동 안 함(데모)

"""Apply one set of measured limits to Nav2 and its collision monitor."""
import math


def configure_navigation(config, directory, measured=None):
    if measured is not None:
        # 실측 필수 값. 하나라도 비었거나(-1) 로봇 반경이 0.20 m(대회 지름 400 mm 의 절반) 초과면 거부
        required = ('robot_radius', 'braking_distance', 'safety_margin',
                    'safety_latency', 'max_wheel_speed', 'max_linear_speed',
                    'max_angular_speed', 'max_linear_accel', 'max_angular_accel')
        # measured.get(k, -1) : 키가 없으면 -1 로 간주
        if any(not math.isfinite(measured.get(k, -1)) or measured.get(k, -1) <= 0
               for k in required) or measured['robot_radius'] > .2:
            raise ValueError('Measure drive limits and expanded radius (<= 0.20 m) first')
        m = measured
        # ① 코스트맵(장애물 지도)의 로봇 크기 — 벽에서 이만큼 떨어져 경로를 짬
        #    nav2.yaml 구조: local_costmap: { local_costmap: { ros__parameters: {...} } }
        for name in ('local_costmap', 'global_costmap'):
            config[name][name]['ros__parameters']['robot_radius'] = m['robot_radius']
        # ② 경로 추종 컨트롤러(Regulated Pure Pursuit) 속도
        controller = config['controller_server']['ros__parameters']['FollowPath']
        controller.update(desired_linear_vel=m['max_linear_speed'],
                          min_approach_linear_velocity=min(.05, m['max_linear_speed']),
                          regulated_linear_scaling_min_speed=min(.1, m['max_linear_speed']),
                          rotate_to_heading_angular_vel=m['max_angular_speed'],
                          max_angular_accel=m['max_angular_accel'])
        # ③ 속도 스무더 — 급가속/급감속을 막는 최종 제한. [x, y, 회전] 순서, y 는 차동구동이라 0
        #    min_velocity 의 x 가 0 → 후진 금지 (motor_bridge 도 후진을 거부함)
        config['velocity_smoother']['ros__parameters'].update(
            max_velocity=[m['max_linear_speed'], 0., m['max_angular_speed']],
            min_velocity=[0., 0., -m['max_angular_speed']],
            max_accel=[m['max_linear_accel'], 0., m['max_angular_accel']],
            max_decel=[-m['max_linear_accel'], 0., -m['max_angular_accel']])
        # The square contains the entire turning envelope in every orientation.
        # ④ Collision Monitor 정지 영역: 이 사각형 안에 스캔 점이 들어오면 즉시 속도 0
        #    반폭 = 로봇 반경 + 제동거리 + 여유 + (반응 지연 동안 최고 속도로 가는 거리)
        stop = (m['robot_radius'] + m['braking_distance'] + m['safety_margin']
                + m['safety_latency'] * m['max_wheel_speed'])
        # 꼭짓점 4개를 [x1,y1, x2,y2, ...] 한 줄로: (+,+) (+,-) (-,-) (-,+)
        config['collision_monitor']['ros__parameters']['Stop']['points'] = [
            stop, stop, stop, -stop, -stop, -stop, -stop, stop]
    # ⑤ 복구 동작은 "기다리기"만 — 후진·제자리 회전 복구는 장애물에 부딪힐 수 있어 뺌
    config['behavior_server']['ros__parameters']['behavior_plugins'] = ['wait']
    # ⑥ 행동 트리(BT): Nav2 가 "경로 계획 → 따라가기 → 실패 시 복구"를 어떤 순서로 할지 적은 XML.
    #    우리 버전(navigate_safe.xml)은 정지·대기·재계획만 사용
    bt = config['bt_navigator']['ros__parameters']
    bt['default_nav_to_pose_bt_xml'] = str(directory / 'navigate_safe.xml')
    bt['default_nav_through_poses_bt_xml'] = str(directory / 'navigate_through_safe.xml')
    return config
