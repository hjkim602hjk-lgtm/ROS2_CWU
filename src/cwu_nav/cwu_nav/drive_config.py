"""Apply one set of measured limits to Nav2 and its collision monitor."""
import math


def configure_navigation(config, directory, measured=None):
    if measured is not None:
        required = ('robot_radius', 'braking_distance', 'safety_margin',
                    'safety_latency', 'max_wheel_speed', 'max_linear_speed',
                    'max_angular_speed', 'max_linear_accel', 'max_angular_accel')
        if any(not math.isfinite(measured.get(k, -1)) or measured.get(k, -1) <= 0
               for k in required) or measured['robot_radius'] > .2:
            raise ValueError('Measure drive limits and expanded radius (<= 0.20 m) first')
        m = measured
        for name in ('local_costmap', 'global_costmap'):
            config[name][name]['ros__parameters']['robot_radius'] = m['robot_radius']
        controller = config['controller_server']['ros__parameters']['FollowPath']
        controller.update(desired_linear_vel=m['max_linear_speed'],
                          min_approach_linear_velocity=min(.05, m['max_linear_speed']),
                          regulated_linear_scaling_min_speed=min(.1, m['max_linear_speed']),
                          rotate_to_heading_angular_vel=m['max_angular_speed'],
                          max_angular_accel=m['max_angular_accel'])
        config['velocity_smoother']['ros__parameters'].update(
            max_velocity=[m['max_linear_speed'], 0., m['max_angular_speed']],
            min_velocity=[0., 0., -m['max_angular_speed']],
            max_accel=[m['max_linear_accel'], 0., m['max_angular_accel']],
            max_decel=[-m['max_linear_accel'], 0., -m['max_angular_accel']])
        # The square contains the entire turning envelope in every orientation.
        stop = (m['robot_radius'] + m['braking_distance'] + m['safety_margin']
                + m['safety_latency'] * m['max_wheel_speed'])
        config['collision_monitor']['ros__parameters']['Stop']['points'] = [
            stop, stop, stop, -stop, -stop, -stop, -stop, stop]
    config['behavior_server']['ros__parameters']['behavior_plugins'] = ['wait']
    bt = config['bt_navigator']['ros__parameters']
    bt['default_nav_to_pose_bt_xml'] = str(directory / 'navigate_safe.xml')
    bt['default_nav_through_poses_bt_xml'] = str(directory / 'navigate_through_safe.xml')
    return config
