# 가상 SLAM 검증 전체를 자동 실행하는 스크립트입니다.
# 데모 시작 → slam_smoke 검사 → Nav2 map saver로 지도 저장 → PGM/YAML 확인 → 데모 종료를 수행합니다.
# 지도와 로그는 /tmp/cwu-slam-check-*에 보관하며 실제 하드웨어는 필요하지 않습니다.

"""Start isolated demo, verify live map/TF, save a map, and stop owned processes."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile

import yaml


def main():
    env = dict(os.environ, ROS_LOCALHOST_ONLY='1')
    env.setdefault('ROS_DOMAIN_ID', '67')
    directory = Path(tempfile.mkdtemp(prefix='cwu-slam-check-'))
    env['ROS_LOG_DIR'] = str(directory / 'ros-log')
    logfile = directory / 'launch.log'
    print(f'Check artifacts: {directory}', flush=True)
    with logfile.open('w') as output:
        launch = subprocess.Popen(
            ['ros2', 'launch', 'cwu_slam', 'demo.launch.py'],
            env=env, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            subprocess.run(['python3', str(Path(__file__).with_name('slam_smoke.py'))],
                           env=env, check=True, timeout=55)
            subprocess.run([
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', str(directory / 'map'), '--ros-args', '-p', 'save_map_timeout:=10.0',
            ], env=env, check=True, timeout=20)
            saved = yaml.safe_load((directory / 'map.yaml').read_text())
            image = directory / saved['image']
            assert saved['resolution'] > 0 and len(saved['origin']) == 3
            assert image.read_bytes().startswith(b'P5') and image.stat().st_size > 100
            assert launch.poll() is None, 'Launch process exited unexpectedly'
            print(f'PASS: saved map YAML and PGM verified in {directory}', flush=True)
        finally:
            if launch.poll() is None:
                # Signal the launcher only; ROS launch forwards SIGINT to its children.
                launch.send_signal(signal.SIGINT)
                try:
                    launch.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(launch.pid, signal.SIGKILL)
                    launch.wait()
            print(logfile.read_text()[-6000:], flush=True)


if __name__ == '__main__':
    main()
