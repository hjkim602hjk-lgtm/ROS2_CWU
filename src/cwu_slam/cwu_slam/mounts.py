# 로봇 본체(base_link) 기준 센서 장착값을 읽어 정적 TF 노드로 만드는 공용 모듈입니다.
# LiDAR와 D415가 같은 mount.yaml을 쓰도록 하여 TF 트리 정의를 한 곳에 둡니다.
# 값 검사와 미보정 경고도 여기서 한 번만 합니다.

"""Shared loader for base_link → sensor static transforms."""
import math

from launch_ros.actions import Node
import yaml

KEYS = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')


def load_mount(path, name):
    """Return the six finite mount values of `name` from the YAML at `path`."""
    with open(path, encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict) or name not in data:
        raise ValueError("%s에 '%s' 항목이 없습니다" % (path, name))
    mount = data[name] or {}
    result = {}
    for key in KEYS:
        if key not in mount:
            raise ValueError("%s의 '%s'에 %s 값이 없습니다" % (path, name, key))
        value = mount[key]
        if isinstance(value, bool) or not math.isfinite(float(value)):
            raise ValueError('Mount values must be finite numbers')
        result[key] = float(value)
    return result


def warn_if_uncalibrated(mount, label):
    """Print a warning when every value is zero, which means "not measured yet"."""
    # 원점 장착 자체는 가능하지만 실물에서 여섯 값이 전부 0이면 미측정이라는 뜻입니다.
    # 그대로 두면 SLAM·인식은 계속 돌면서 조용히 어긋난 결과를 냅니다.
    if any(mount[key] for key in KEYS):
        return
    print('[cwu_slam] 경고: %s 장착값이 전부 0입니다 (미보정 기본값). '
          'base_link 기준 위치·각도를 측정해 입력하십시오.' % label)


def static_tf_node(name, child_frame, mount, use_sim_time, parent_frame='base_link'):
    """Build the tf2_ros static publisher for one measured sensor mount."""
    args = []
    for key in KEYS:
        args.extend(['--' + key, str(mount[key])])
    return Node(
        package='tf2_ros', executable='static_transform_publisher', name=name,
        arguments=args + ['--frame-id', parent_frame, '--child-frame-id', child_frame],
        parameters=[{'use_sim_time': use_sim_time}], output='screen')
