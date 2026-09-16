# cwu_nav Python 패키지의 설치 구성을 정의합니다.
# launch·YAML을 ROS 패키지 공유 경로에 설치하고 target_follower 실행 명령을 등록합니다.

from glob import glob
from setuptools import find_packages, setup

setup(
    name='cwu_nav', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/cwu_nav']),
        ('share/cwu_nav', ['package.xml']),
        ('share/cwu_nav/launch', glob('launch/*.launch.py')),
        ('share/cwu_nav/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='CWU Robot Team', maintainer_email='maintainer@example.com',
    description='Nav2 bringup and the visual servo for the competition arena', license='Apache-2.0',
    tests_require=['pytest'], entry_points={'console_scripts': [
        'target_follower = cwu_nav.target_follower:main']},
)
