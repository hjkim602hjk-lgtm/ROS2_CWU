# cwu_slam Python 패키지의 설치 구성을 정의합니다.
# launch·YAML·RViz 파일을 ROS 패키지 공유 경로에 설치하고 demo_sensors 실행 명령을 등록합니다.

from glob import glob
from setuptools import find_packages, setup

setup(
    name='cwu_slam', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/cwu_slam']),
        ('share/cwu_slam', ['package.xml']),
        ('share/cwu_slam/launch', glob('launch/*.launch.py')),
        ('share/cwu_slam/config', glob('config/*.yaml')),
        ('share/cwu_slam/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='CWU Robot Team', maintainer_email='maintainer@example.com',
    description='Encoder-assisted 2D SLAM bringup', license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['demo_sensors = cwu_slam.demo_sensors:main']},
)
