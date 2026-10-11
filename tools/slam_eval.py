#!/usr/bin/env python3
"""Score a SLAM run against ICP reference motions (PC, offline).

    python3 tools/slam_eval.py RUN_BAG PAIRS_JSON [--png map.png]

RUN_BAG must hold /tf with map->odom (SLAM) and odom->base_link (odometry); the
recorded robot bag and tools/slam_replay.py outputs both do. For every reference
pair (t0, t1, ICP motion) from tools/odom_calib.py, the SLAM pose change between
t0 and t1 (in the t0 base frame) is compared with ICP; odometry alone is scored
the same way as a baseline. Lower is better. --png writes the last /map.
"""
import argparse
import json
import math

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


def read(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    series = {'map': [], 'odom': []}
    last_map = None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/tf':
            for t in deserialize_message(data, get_message(types[topic])).transforms:
                key = {('map', 'odom'): 'map', ('odom', 'base_link'): 'odom'}.get(
                    (t.header.frame_id, t.child_frame_id))
                if key:
                    q, p = t.transform.rotation, t.transform.translation
                    series[key].append((t.header.stamp.sec + t.header.stamp.nanosec * 1e-9,
                                        p.x, p.y, 2 * math.atan2(q.z, q.w)))
        elif topic == '/map':
            last_map = deserialize_message(data, get_message(types[topic]))
    out = {}
    for k, v in series.items():
        a = np.array(sorted(v)) if v else np.zeros((0, 4))
        if len(a):
            a[:, 3] = np.unwrap(a[:, 3])
        out[k] = a
    return out, last_map


def at(a, t):
    """Pose at t: interpolated, or the latest earlier one (map->odom is piecewise constant)."""
    if len(a) == 0 or t < a[0, 0]:
        return None
    if t > a[-1, 0]:
        return tuple(a[-1, 1:])
    return tuple(np.interp(t, a[:, 0], a[:, c]) for c in (1, 2, 3))


def compose(p, q):
    x, y, th = p
    c, s = math.cos(th), math.sin(th)
    return (x + c * q[0] - s * q[1], y + s * q[0] + c * q[1], th + q[2])


def relative(p0, p1):
    c, s = math.cos(p0[2]), math.sin(p0[2])
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    return (c * dx + s * dy, -s * dx + c * dy, math.remainder(p1[2] - p0[2], 2 * math.pi))


def score(rel_fn, pairs):
    et, er = [], []
    for p in pairs:
        r = rel_fn(p['t0'], p['t1'])
        if r is None:
            continue
        x, y, th = p['icp']
        et.append(math.hypot(r[0] - x, r[1] - y))
        er.append(abs(math.degrees(math.remainder(r[2] - th, 2 * math.pi))))
    if not et:
        return None
    return len(et), 100 * float(np.sqrt(np.mean(np.square(et)))), float(np.sqrt(np.mean(np.square(er))))


def save_png(m, path):
    from PIL import Image
    d = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    img = np.full(d.shape, 205, np.uint8)
    img[d == 0], img[d > 50] = 254, 0
    Image.fromarray(img[::-1]).save(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_bag')
    ap.add_argument('pairs')
    ap.add_argument('--png')
    a = ap.parse_args()
    series, last_map = read(a.run_bag)
    pairs = json.load(open(a.pairs))['pairs']

    def slam_rel(t0, t1):
        poses = []
        for t in (t0, t1):
            mo, ob = at(series['map'], t), at(series['odom'], t)
            if mo is None or ob is None:
                return None
            poses.append(compose(mo, ob))
        return relative(*poses)

    def odom_rel(t0, t1):
        p0, p1 = at(series['odom'], t0), at(series['odom'], t1)
        return None if p0 is None or p1 is None else relative(p0, p1)

    for name, fn in (('SLAM', slam_rel), ('odom', odom_rel)):
        s = score(fn, pairs)
        print(f'{name:4s}: ' + ('평가 불가 (TF 없음)' if s is None else
                                f'쌍 {s[0]}개, 위치오차 RMS {s[1]:.1f} cm, 회전오차 RMS {s[2]:.2f}°'))
    if a.png and last_map is not None:
        save_png(last_map, a.png)
        print(f'지도 저장: {a.png} ({last_map.info.width}x{last_map.info.height})')


if __name__ == '__main__':
    main()
