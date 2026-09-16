# cwu_base Python 패키지의 설치 구성을 정의합니다.
# launch·YAML을 ROS 패키지 공유 경로에 설치하고 encoder_odom 실행 명령을 등록합니다.

from glob import glob
from setuptools import find_packages, setup

setup(
    name='cwu_base', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/cwu_base']),
        ('share/cwu_base', ['package.xml']),
        ('share/cwu_base/launch', glob('launch/*.launch.py')),
        ('share/cwu_base/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='CWU Robot Team', maintainer_email='maintainer@example.com',
    description='Differential-drive encoder odometry', license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['encoder_odom = cwu_base.encoder_odom:main']},
)
