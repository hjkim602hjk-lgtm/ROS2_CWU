# 브라우저로 SLAM 상태를 보는 웹 UI를 실행하는 launch 진입점입니다.
# rosbridge WebSocket 서버(ws_port)와 index.html 정적 파일 서버(http_port)를 함께 띄웁니다.
# SLAM과 별개 프로세스라 slam.launch.py가 떠 있는 중에 따로 켜고 끌 수 있습니다.

"""Serve the browser monitoring page and the rosbridge socket it talks to."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    web = Path(get_package_share_directory('cwu_slam')) / 'web'
    rosbridge = Path(get_package_share_directory('rosbridge_server')) / 'launch'
    return LaunchDescription([
        # rosbridge의 XML launch가 선언하는 'port'는 부모 스코프로 새어 나와 같은
        # 이름의 인자를 덮어씁니다. 그래서 우리 인자는 이름을 달리 둡니다.
        DeclareLaunchArgument('http_port', default_value='8080'),
        DeclareLaunchArgument('ws_port', default_value='9090'),
        # README에도 적혀 있지만 띄우는 순간에 보여야 하는 경고입니다. rosbridge는
        # 인증 없이 토픽 발행과 서비스 호출까지 허용하므로 원격 조종 통로가 됩니다.
        LogInfo(msg='[cwu_slam] 경고: 웹 UI는 개발·점검용입니다. 미션 시작 전에 반드시 '
                    '끄십시오 (대회 규정: 미션 중 외부 통신·원격 제어 금지).'),
        # rosbridge의 launch는 XML이라 Python 전용 소스로는 열리지 않습니다.
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(str(rosbridge / 'rosbridge_websocket_launch.xml')),
            launch_arguments={'port': LaunchConfiguration('ws_port')}.items()),
        # 페이지가 정적 파일 한 장이라 표준 라이브러리 서버로 충분합니다.
        # 직접 만든 서버 코드가 없으면 유지보수할 것도 없습니다.
        ExecuteProcess(cmd=[
            'python3', '-m', 'http.server', LaunchConfiguration('http_port'),
            '--bind', '0.0.0.0', '--directory', str(web),
        ], output='screen'),
    ])
