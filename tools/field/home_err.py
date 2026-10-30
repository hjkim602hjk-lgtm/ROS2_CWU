"""ICP between saved start/end scans -> physical return error. home_err.py START END"""
import json, math, sys
import numpy as np
sys.path.insert(0, __import__('os').path.join(__import__('os').path.dirname(__import__('os').path.abspath(__file__)), '..'))  # tools/
from odom_calib import icp, points
YAW = 3.1381
def load(p):
    d = json.load(open(p))
    return d, (0, np.array(d['ranges']), d['angle_min'], d['inc'], d['range_min'], 0)
a, sa = load(sys.argv[1]); b, sb = load(sys.argv[2])
g0 = b['slam'] or [0.0, 0.0, 0.0]  # SLAM pose as the first guess, then a heading sweep around it
if a['slam'] and b['slam']:
    c, si = math.cos(a['slam'][2]), math.sin(a['slam'][2]); dx, dy = b['slam'][0] - a['slam'][0], b['slam'][1] - a['slam'][1]
    g0 = [c*dx + si*dy, -si*dx + c*dy, b['slam'][2] - a['slam'][2]]
cands = [icp(points(sa, YAW), points(sb, YAW), (g0[0], g0[1], g0[2] + math.radians(g))) for g in range(-40, 41, 5)]
(x, y, th), inl, rms, _ = max((c for c in cands if c), key=lambda c: c[1])
print(f'실제(스캔 정합) 복귀 오차: {math.hypot(x, y)*100:.1f} cm (x {x*100:+.1f}, y {y*100:+.1f}), 방향 {math.degrees(th):+.1f}°  [inlier {inl:.2f}, rms {rms*100:.1f} cm]')
if b['slam']:
    print(f'SLAM이 본 최종 위치: ({b["slam"][0]*100:+.1f}, {b["slam"][1]*100:+.1f}) cm, {math.degrees(b["slam"][2]):+.1f}°')
