"""Trial bag timeline: map->odom jumps and per-pair SLAM-vs-ICP errors. timeline.py BAG PAIRS_JSON"""
import json, math, sys
sys.path.insert(0, __import__('os').path.join(__import__('os').path.dirname(__import__('os').path.abspath(__file__)), '..'))  # tools/
import slam_eval as se
series, _ = se.read(sys.argv[1])
mo = series['map']; t0 = series['odom'][0, 0]
print(f'map->odom 메시지 {len(mo)}개')
prev = None
for row in mo:
    if prev is not None:
        d = math.hypot(row[1] - prev[1], row[2] - prev[2]); a = math.degrees(abs(row[3] - prev[3]))
        if d > 0.03 or a > 2.0:
            print(f'  {row[0]-t0:6.1f}s  map->odom 점프 {d*100:5.1f} cm, {a:5.1f}°')
    prev = row
if len(mo):
    print(f'최종 map->odom: ({mo[-1,1]*100:+.1f}, {mo[-1,2]*100:+.1f}) cm, {math.degrees(mo[-1,3]):+.1f}°')
pairs = json.load(open(sys.argv[2]))['pairs']
bad = []
for p in pairs:
    poses = []
    for t in (p['t0'], p['t1']):
        a, b = se.at(series['map'], t), se.at(series['odom'], t)
        poses.append(None if a is None or b is None else se.compose(a, b))
    if None in poses: continue
    s = se.relative(*poses); o = se.relative(se.at(series['odom'], p['t0']), se.at(series['odom'], p['t1'])); i = p['icp']
    es = math.degrees(math.remainder(s[2] - i[2], 2*math.pi)); eo = math.degrees(math.remainder(o[2] - i[2], 2*math.pi))
    ts = math.hypot(s[0]-i[0], s[1]-i[1]); to = math.hypot(o[0]-i[0], o[1]-i[1])
    if abs(es) > 3 or ts > 0.05 or abs(eo) > 3 or to > 0.05:
        bad.append(f'  {p["t0"]-t0:6.1f}s  SLAM 오차 {ts*100:4.1f} cm {es:+5.1f}° | odom 오차 {to*100:4.1f} cm {eo:+5.1f}° | ICP 이동 {math.hypot(*i[:2])*100:.0f} cm {math.degrees(i[2]):+.0f}°')
print(f'큰 오차 구간 {len(bad)}/{len(pairs)}:'); print('\n'.join(bad[:30]))
