#!/bin/bash
# Gazebo 랜덤 경기장에서 미션 1 한 판 (PC 전용, 도메인 78 — 실물 30과 분리).
#   tools/sim/run_maze.sh SEED [제한 s=600] [gui]     # SEED = example 이면 규정 그림 1 예시 배치
#   SIM_CELLS='3,8 3,9 7,1' tools/sim/run_maze.sh custom  # 직접 정한 장애물 칸
#   SIM_CELLS="$(cat tools/sim/maps/m1.txt)" SIM_NUM=1 SIM_TITLE='맵 1' tools/sim/run_maze.sh custom  # 사용자 맵 (10-31)
#   결과 한 줄: seed, 끝난 이유, 시뮬레이션 시각, 큐브 보유, 출발 칸 중심까지 실제 거리, 막힘 판정 수
#   로그·world·경로: ${SIM_OUT:-/tmp/cwu_sim}/maze_SEED.{log,world,png}, track_SEED.txt
cd "$(dirname "$0")/../.." || exit 1
seed=$1 limit=${2:-600} gui=false out=${SIM_OUT:-/tmp/cwu_sim}
[ "$3" = gui ] && gui=true
mkdir -p "$out"
# 병렬 실행: 시드마다 다른 ROS 도메인·Gazebo 포트 (80~99)
flag=(); num=$seed
[ "$seed" = example ] && flag=(--example) num=0
[ "$seed" = custom ] && flag=(--cells "$SIM_CELLS") num=0  # SIM_CELLS="3,8 3,9 ..." 직접 정한 장애물 칸
num=${SIM_NUM:-$num}  # 같은 맵(custom)을 동시에 여러 판: SIM_NUM으로 도메인을 나눔
export ROS_DOMAIN_ID=$((80 + num % 20)) GAZEBO_MODEL_DATABASE_URI=
export GAZEBO_MASTER_URI=http://localhost:$((11345 + ROS_DOMAIN_ID))
source /opt/ros/humble/setup.bash
source install/setup.bash
# 칸 목록은 공백이 있어 배열로 넘김 (따옴표 없이 넘겨 world를 못 만들고 9시간 멈춘 적 있음, 2026-10-30)
ros2 run cwu_sim make_maze --seed "$num" "${flag[@]}" --out "$out/maze_$seed.world" > "$out/maze_$seed.txt" \
  || { echo "seed $seed | world 생성 실패: $(tail -1 "$out/maze_$seed.txt")"; exit 1; }
setsid ros2 launch cwu_sim maze.launch.py world:="$out/maze_$seed.world" gui:=$gui track:="$out/track_$seed.txt" \
  > "$out/maze_$seed.log" 2>&1 &
group=$!
for _ in $(seq "$limit"); do
  sleep 1
  grep -q '미션 종료' "$out/maze_$seed.log" && break
  kill -0 $group 2> /dev/null || break
done
# 끝 위치 = 기록 파일 마지막 줄 (world 원점 = 출발 자세라 출발 기준). gz model -p는 큐브를 지운 뒤 응답이 끊겼음
pose=$(grep -v "^E" "$out/track_$seed.txt" 2> /dev/null | tail -1 | cut -d" " -f2-)
kill -INT -- -$group 2> /dev/null; sleep 5; kill -9 -- -$group 2> /dev/null
end=$(grep -o '미션 종료 ([^)]*)' "$out/maze_$seed.log" | tail -1)
line="seed $seed | ${end:-시간 초과/중단} | $(python3 tools/sim/blocks.py "$seed" "$out/maze_$seed.log" "$pose")"
echo "$line"
# 경로 그림 (기록 노드 track_recorder의 /odom·상태 전이 → maze_SEED.png)
/usr/bin/python3 tools/sim/plot_track.py "$seed" "$out/track_$seed.txt" "$line" "$out/maze_$seed.png" > /dev/null 2>&1
