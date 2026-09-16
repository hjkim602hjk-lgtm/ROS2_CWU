# 가상 시리얼 포트로 엔코더 노드 전체 경로를 하드웨어 없이 검증하는 스크립트입니다.
# 가짜 틱을 흘려 넣고 /odom 값과 odom → base_link TF가 맞게 나오는지 확인합니다.
# 실제 Nucleo 없이 파싱·되감김·적분·발행 연결을 한 번에 점검할 때 씁니다.

"""Feed synthetic ticks through a pty and verify the published odometry and TF."""
import math
import os
import pty
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import yaml

RADIUS, SEPARATION, TICKS_PER_REV = 0.0325, 0.2, 1000.0
DISTANCE = 1.0  # 1 m 직진을 흉내 냅니다.


def write_config(directory):
    """실측값이 채워진 상태를 흉내 낸 임시 설정 파일을 만듭니다."""
    path = directory / 'encoder.yaml'
    path.write_text(yaml.safe_dump({'encoder_odom': {'ros__parameters': {
        'baud': 115200, 'left_field': 0, 'right_field': 1,
        'wheel_radius': RADIUS, 'wheel_separation': SEPARATION,
        'ticks_per_rev': TICKS_PER_REV, 'counter_bits': 16,
    }}}))
    return path


def main():
    directory = Path(tempfile.mkdtemp(prefix='cwu-encoder-check-'))
    env = dict(os.environ, ROS_LOCALHOST_ONLY='1')
    env.setdefault('ROS_DOMAIN_ID', '67')
    env['ROS_LOG_DIR'] = str(directory / 'ros-log')
    print(f'Check artifacts: {directory}', flush=True)

    master, slave = pty.openpty()
    logfile = directory / 'node.log'
    with logfile.open('w') as output:
        node = subprocess.Popen([
            'ros2', 'run', 'cwu_base', 'encoder_odom', '--ros-args',
            '--params-file', str(write_config(directory)),
            '-p', f'port:={os.ttyname(slave)}',
        ], env=env, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            time.sleep(4)  # 노드가 뜨고 시리얼을 열 시간을 줍니다.
            ticks = round(DISTANCE / (2 * math.pi * RADIUS) * TICKS_PER_REV)
            # 최대값 근처에서 시작해 되감김까지 함께 지나가게 합니다.
            start = (1 << 16) - ticks // 2
            for i in range(0, ticks + 1, max(ticks // 50, 1)):
                value = (start + i) % (1 << 16)
                os.write(master, f'{value},{value}\n'.encode())
                time.sleep(0.02)

            # 주행이 끝난 뒤에 구독해야 마지막 자세를 받습니다. 노드는 틱이 올 때만
            # 발행하므로, 구독이 붙은 다음 같은 값을 한 번 더 흘려 발행을 유발합니다.
            echoes = [subprocess.Popen(['ros2', 'topic', 'echo', topic, '--once'],
                                       env=env, stdout=subprocess.PIPE, text=True)
                      for topic in ('/odom', '/tf')]
            time.sleep(3)
            os.write(master, f'{value},{value}\n'.encode())
            message, transform = (
                next(iter(yaml.safe_load_all(e.communicate(timeout=20)[0])))
                for e in echoes)
            x = message['pose']['pose']['position']['x']
            assert abs(x - DISTANCE) < 0.02, f'x={x}, expected about {DISTANCE}'
            assert abs(message['pose']['pose']['position']['y']) < 1e-6
            assert abs(message['pose']['pose']['orientation']['z']) < 1e-6
            assert message['header']['frame_id'] == 'odom'
            assert message['child_frame_id'] == 'base_link'

            sent, = transform['transforms']
            assert (sent['header']['frame_id'], sent['child_frame_id']) == \
                ('odom', 'base_link'), sent
            assert abs(sent['transform']['translation']['x'] - x) < 1e-6
            assert node.poll() is None, 'Node exited unexpectedly'
            print(f'PASS: /odom x={x:.3f} m after a wrapping 1 m run, '
                  'odom -> base_link TF matches', flush=True)
        finally:
            if node.poll() is None:
                node.send_signal(signal.SIGINT)
                try:
                    node.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(node.pid, signal.SIGKILL)
                    node.wait()
            os.close(master)
            os.close(slave)
            print(logfile.read_text()[-4000:], flush=True)


if __name__ == '__main__':
    main()
