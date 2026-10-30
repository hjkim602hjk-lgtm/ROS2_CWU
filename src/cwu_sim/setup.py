# cwu_sim: PC 전용 Gazebo 시뮬레이션 (랜덤 경기장 생성 + 가짜 보드).
from glob import glob
from setuptools import find_packages, setup

setup(
    name='cwu_sim', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/cwu_sim']),
        ('share/cwu_sim', ['package.xml']),
        ('share/cwu_sim/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='CWU Robot Team', maintainer_email='maintainer@example.com',
    description='Gazebo simulation for mission 1 (maze) and mission 2 (battle)', license='Apache-2.0',
    tests_require=['pytest'], entry_points={'console_scripts': [
        'make_maze = cwu_sim.maze:main',
        'fake_board = cwu_sim.fake_board:main',
        'track_recorder = cwu_sim.track_recorder:main',
        'make_battle = cwu_sim.battle:main',
        'referee = cwu_sim.referee:main']},
)
