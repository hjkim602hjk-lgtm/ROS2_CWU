#!/usr/bin/env python3
"""Calibrate wheel odometry against LiDAR scan matching from a recorded bag (runs on the PC).

    python3 tools/odom_calib.py BAG [--sep 0.20686] [--radius 0.04151] [--pairs out.json]

For scan pairs that moved >= MIN_MOVE m or turned >= MIN_TURN, point-to-line ICP
(initialised from odometry) gives the motion the LiDAR saw. With odometry's wheel
travel s_l, s_r per pair, least squares solves the true left/right travel scales
a_l, a_r and track width b from
    chord_icp / sinc(theta/2) = (a_l s_l + a_r s_r) / 2,   theta_icp * b = a_r s_r - a_l s_l.
New wheel_radius = r * (a_l + a_r) / 2. Pairs where ICP is weak (few inliers, large
residual, or a corridor-like degenerate direction) are discarded.
Also reports the LiDAR yaw offset (ICP vs odometry motion direction) and the
scan-time offset that best explains the data.
"""
import argparse
import json
import math

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial import cKDTree
import rosbag2_py

MIN_MOVE, MAX_MOVE = 0.20, 0.45      # m
MIN_TURN, MAX_TURN = math.radians(25), math.radians(70)


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    odom, scans, mount_yaw = [], [], None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/odom':
            m = deserialize_message(data, get_message(types[topic]))
            q = m.pose.pose.orientation
            odom.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                         m.pose.pose.position.x, m.pose.pose.position.y,
                         2 * math.atan2(q.z, q.w)))
        elif topic == '/scan':
            m = deserialize_message(data, get_message(types[topic]))
            scans.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                          np.array(m.ranges, dtype=float), m.angle_min, m.angle_increment,
                          m.range_min, m.scan_time))
        elif topic == '/tf_static' and mount_yaw is None:
            m = deserialize_message(data, get_message(types[topic]))
            for t in m.transforms:
                if t.child_frame_id == 'laser_frame':
                    q = t.transform.rotation
                    mount_yaw = 2 * math.atan2(q.z, q.w)
    odom = np.array(odom)
    odom = odom[np.argsort(odom[:, 0])]
    odom[:, 3] = np.unwrap(odom[:, 3])
    return odom, scans, mount_yaw


def odom_at(odom, t):
    """Interpolated (x, y, unwrapped yaw) at time t, or None outside the log."""
    if t < odom[0, 0] or t > odom[-1, 0]:
        return None
    return tuple(np.interp(t, odom[:, 0], odom[:, c]) for c in (1, 2, 3))


def wheel_travel(odom, t0, t1, sep):
    """Signed path length and heading change along odometry, split into wheel travel."""
    k = (odom[:, 0] > t0) & (odom[:, 0] < t1)
    a, b = odom_at(odom, t0), odom_at(odom, t1)
    xs, ys, th = (np.r_[a[c], odom[k, c + 1], b[c]] for c in (0, 1, 2))
    ds = np.diff(xs) * np.cos(th[:-1]) + np.diff(ys) * np.sin(th[:-1])
    dist, dth = ds.sum(), b[2] - a[2]
    return dist - dth * sep / 2, dist + dth * sep / 2


def points(scan, yaw):
    _, r, a0, inc, rmin, _ = scan
    a = a0 + np.arange(len(r)) * inc + yaw
    k = (r > max(rmin, 0.15)) & (r < 8.0)
    return np.c_[r[k] * np.cos(a[k]), r[k] * np.sin(a[k])]


def normals(pts, tree):
    """Unit normals from the 6 nearest neighbours; NaN where the patch is not line-like."""
    _, idx = tree.query(pts, k=6, distance_upper_bound=0.15)
    out = np.full_like(pts, np.nan)
    for i, nb in enumerate(idx):
        nb = nb[nb < len(pts)]
        if len(nb) < 4:
            continue
        p = pts[nb] - pts[nb].mean(axis=0)
        w, v = np.linalg.eigh(p.T @ p)
        if w[0] < 0.1 * w[1]:  # clearly a line
            out[i] = v[:, 0]
    return out


def icp(A, B, guess, iters=40):
    """Point-to-line ICP: pose (x, y, th) of B's frame in A's frame, plus quality stats."""
    tree = cKDTree(A)
    nA = normals(A, tree)
    x, y, th = guess
    H, used = np.eye(3), 0
    for it in range(iters):
        thr = max(0.05, 0.3 * (1 - it / 15))
        c, s = math.cos(th), math.sin(th)
        P = B @ np.array([[c, s], [-s, c]]) + [x, y]
        d, j = tree.query(P, distance_upper_bound=thr)
        jj = np.minimum(j, len(A) - 1)
        k = np.isfinite(d) & np.isfinite(nA[jj, 0])
        used = int(k.sum())
        if used < 30:
            return None
        p, q, n = P[k], A[jj[k]], nA[jj[k]]
        # R(th + dth) B + t + dt  ~  p + dth * perp(p - t) + dt ;  residual along the normal
        J = np.c_[n[:, 0], n[:, 1], -n[:, 0] * (p[:, 1] - y) + n[:, 1] * (p[:, 0] - x)]
        e = np.einsum('ij,ij->i', n, q - p)
        H = J.T @ J
        delta = np.linalg.solve(H + 1e-9 * np.eye(3), J.T @ e)
        x, y, th = x + delta[0], y + delta[1], th + delta[2]
        if np.abs(delta).max() < 1e-5:
            break
    c, s = math.cos(th), math.sin(th)
    d, _ = tree.query(B @ np.array([[c, s], [-s, c]]) + [x, y])
    inlier = d < 0.05
    rms = float(np.sqrt(np.mean(d[inlier] ** 2))) if inlier.any() else 1.0
    # Smallest translational information per point: ~0 in a corridor (unobservable axis).
    eig = float(np.linalg.eigvalsh(H[:2, :2] / used)[0])
    return (x, y, th), float(inlier.mean()), rms, eig


def collect(odom, scans, yaw, sep, dt):
    pairs, i = [], 0
    while i < len(scans) - 1:
        ti = scans[i][0] + dt
        pi = odom_at(odom, ti)
        j, found = i + 1, None
        while j < len(scans) and pi is not None:
            pj = odom_at(odom, scans[j][0] + dt)
            if pj is None:
                break
            move, turn = math.hypot(pj[0] - pi[0], pj[1] - pi[1]), abs(pj[2] - pi[2])
            if move > MAX_MOVE or turn > MAX_TURN:
                break
            if move >= MIN_MOVE or turn >= MIN_TURN:
                found = (j, pj)
                break
            j += 1
        if found is None:
            i += 1
            continue
        j, pj = found
        c, s = math.cos(pi[2]), math.sin(pi[2])
        dx, dy = pj[0] - pi[0], pj[1] - pi[1]
        guess = (c * dx + s * dy, -s * dx + c * dy, pj[2] - pi[2])
        res = icp(points(scans[i], yaw), points(scans[j], yaw), guess)
        if res is not None:
            pose, inl, rms, eig = res
            sl, sr = wheel_travel(odom, ti, scans[j][0] + dt, sep)
            pairs.append(dict(t0=ti, t1=scans[j][0] + dt, odom=guess, icp=pose,
                              s_l=sl, s_r=sr, inlier=inl, rms=rms, eig=eig))
        i = j  # non-overlapping pairs: independent samples
    return pairs


def solve(pairs):
    rows, rhs = [], []
    for p in pairs:
        x, y, th = p['icp']
        chord = math.hypot(x, y) * (1 if p['s_l'] + p['s_r'] >= 0 else -1)
        sinc = math.sin(th / 2) / (th / 2) if abs(th) > 1e-6 else 1.0
        rows.append([p['s_l'] / 2, p['s_r'] / 2, 0.0])
        rhs.append(chord / sinc)
        rows.append([p['s_l'], -p['s_r'], th])  # heading equation, in metres
        rhs.append(0.0)
    sol, *_ = np.linalg.lstsq(np.array(rows), np.array(rhs), rcond=None)
    return sol


def good(p):
    """ICP quality, and no wheel slip: ICP motion within 30% of odometry.

    Pushing against something below the LiDAR (170 mm) spins the wheels while the
    scan stays still (2026-10-11 bag: odom 0.20 m, ICP 0.00 m). Such pairs would
    drag the scale toward zero, so they are dropped, not down-weighted.
    """
    if not (p['inlier'] > 0.6 and p['rms'] < 0.03 and p['eig'] > 0.05):
        return False
    od, ic = math.hypot(*p['odom'][:2]), math.hypot(*p['icp'][:2])
    if od > 0.1 and not 0.7 < ic / od < 1.3:
        return False
    if abs(p['odom'][2]) > math.radians(15) and not 0.7 < p['icp'][2] / p['odom'][2] < 1.3:
        return False
    return True


def fit(pairs):
    """Least squares, then refit once without pairs whose residual exceeds 3x the median."""
    a_l, a_r, b = solve(pairs)
    res = []
    for p in pairs:
        x, y, th = p['icp']
        sinc = math.sin(th / 2) / (th / 2) if abs(th) > 1e-6 else 1.0
        dist = (a_l * p['s_l'] + a_r * p['s_r']) / 2
        res.append(abs(math.hypot(x, y) / sinc - abs(dist)) + abs(th - (a_r * p['s_r'] - a_l * p['s_l']) / b) * b)
    keep = [p for p, r in zip(pairs, res) if r <= 3 * float(np.median(res)) + 1e-3]
    return solve(keep), keep


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--sep', type=float, default=0.20686)
    ap.add_argument('--radius', type=float, default=0.04151)
    ap.add_argument('--dt', type=float, default=None, help='scan stamp offset; default: search')
    ap.add_argument('--yaw', type=float, default=None, help='laser mount yaw; default: from bag')
    ap.add_argument('--pairs', default=None)
    a = ap.parse_args()
    odom, scans, bag_yaw = read_bag(a.bag)
    yaw = a.yaw if a.yaw is not None else bag_yaw
    print(f'odom {len(odom)}개, scan {len(scans)}개, {odom[-1, 0] - odom[0, 0]:.0f} s, '
          f'mount yaw {math.degrees(yaw):.1f}°')
    best = None
    for dt in ([a.dt] if a.dt is not None else [-0.05, 0.0, 0.05, 0.10, 0.15]):
        allp = collect(odom, scans, yaw, a.sep, dt)
        pairs = [p for p in allp if good(p)]
        (a_l, a_r, b), pairs = fit(pairs)
        # Residual after calibration: how well one (a_l, a_r, b) explains every pair.
        res = []
        for p in pairs:
            th = (a_r * p['s_r'] - a_l * p['s_l']) / b
            res.append(math.remainder(p['icp'][2] - th, 2 * math.pi))
        rot_rms = math.degrees(float(np.sqrt(np.mean(np.square(res)))))
        print(f'dt {dt:+.2f} s: 쌍 {len(allp)}개 중 품질 통과 {len(pairs)}개, 보정 후 회전 잔차 RMS {rot_rms:.2f}°')
        if pairs and (best is None or rot_rms < best[0]):
            best = (rot_rms, dt, pairs, (a_l, a_r, b))
    _, dt, pairs, (a_l, a_r, b) = best
    straight = [p for p in pairs if abs(p['icp'][2]) < math.radians(8) and math.hypot(*p['icp'][:2]) > 0.15]
    spins = [p for p in pairs if math.hypot(*p['odom'][:2]) < 0.03]
    print(f'\n선택 dt {dt:+.2f} s, 쌍 {len(pairs)}개 (직진형 {len(straight)}, 제자리회전 {len(spins)})')
    print(f'왼쪽 바퀴 배율 a_l = {a_l:.4f}, 오른쪽 a_r = {a_r:.4f}, 바퀴 간격 b = {b:.4f} m')
    print(f'→ wheel_radius     {a.radius:.5f} → {a.radius * (a_l + a_r) / 2:.5f}  (좌우 차 {100 * (a_l - a_r) / ((a_l + a_r) / 2):+.2f}%)')
    print(f'→ wheel_separation {a.sep:.5f} → {b:.5f}')
    if straight:
        off = [math.remainder(math.atan2(p['icp'][1], p['icp'][0]) - math.atan2(p['odom'][1], p['odom'][0]),
                              2 * math.pi) for p in straight]
        d = np.array([math.hypot(*p['icp'][:2]) / max(math.hypot(*p['odom'][:2]), 1e-6) for p in straight])
        print(f'직진형: ICP/odom 거리비 중앙값 {np.median(d):.4f}, 이동방향 차 중앙값 {math.degrees(np.median(off)):+.2f}°'
              f' (라이다 yaw 보정 후보)')
    if spins:
        r = np.array([p['icp'][2] / p['odom'][2] for p in spins if abs(p['odom'][2]) > 0.2])
        print(f'제자리회전: ICP/odom 회전비 중앙값 {np.median(r):.4f} (n={len(r)})')
    if a.pairs:
        with open(a.pairs, 'w') as f:
            json.dump(dict(dt=dt, yaw=yaw, a_l=a_l, a_r=a_r, sep=b, radius=a.radius, sep0=a.sep,
                           pairs=pairs), f, indent=1)


if __name__ == '__main__':
    main()
