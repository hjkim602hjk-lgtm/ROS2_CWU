"""Measured drive limits must also constrain planning and collision stopping."""
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from cwu_nav.drive_config import configure_navigation

CONFIG = Path(__file__).parents[1] / 'config'


def test_measured_limits_cover_rotation_and_stopping():
    config = yaml.safe_load((CONFIG / 'nav2.yaml').read_text())
    measured = dict(robot_radius=.18, braking_distance=.04, safety_margin=.02,
                    safety_latency=.9, max_wheel_speed=.15, max_linear_speed=.1,
                    max_angular_speed=.5, max_linear_accel=.2, max_angular_accel=.4)
    configure_navigation(config, CONFIG, measured)
    p = config['collision_monitor']['ros__parameters']
    assert p['cmd_vel_in_topic'] == '/cmd_vel'
    assert p['cmd_vel_out_topic'] == '/cmd_vel_safe'
    assert min(abs(v) for v in p['Stop']['points']) >= .375 - 1e-9
    assert config['local_costmap']['local_costmap']['ros__parameters']['robot_radius'] == .18
    assert config['velocity_smoother']['ros__parameters']['max_velocity'] == [.1, 0., .5]
    assert config['velocity_smoother']['ros__parameters']['min_velocity'][0] == 0.
    for key in ('default_nav_to_pose_bt_xml', 'default_nav_through_poses_bt_xml'):
        tree = ET.parse(config['bt_navigator']['ros__parameters'][key])
        assert not any(n.tag in ('BackUp', 'Spin', 'DriveOnHeading') for n in tree.iter())
    assert config['behavior_server']['ros__parameters']['behavior_plugins'] == ['wait']


def test_unmeasured_stop_geometry_rejected():
    with pytest.raises(ValueError):
        configure_navigation({}, CONFIG, dict(robot_radius=-1.))
