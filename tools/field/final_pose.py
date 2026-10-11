"""Final SLAM pose (map->base_link at the last odom stamp) of a run bag vs a truth pose.
final_pose.py RUN_BAG truth_x truth_y truth_deg"""
import math, sys
sys.path.insert(0, __import__('os').path.join(__import__('os').path.dirname(__import__('os').path.abspath(__file__)), '..'))  # tools/
import slam_eval as se
series, _ = se.read(sys.argv[1])
t = series['odom'][-1, 0]
p = se.compose(se.at(series['map'], t), se.at(series['odom'], t))
tx, ty, td = map(float, sys.argv[2:5])
print(f'최종 SLAM ({p[0]*100:+.1f}, {p[1]*100:+.1f}) cm {math.degrees(p[2]):+.1f}° | 실제 대비 '
      f'{math.hypot(p[0]-tx, p[1]-ty)*100:.1f} cm, {math.degrees(math.remainder(p[2]-math.radians(td), 2*math.pi)):+.1f}°')
