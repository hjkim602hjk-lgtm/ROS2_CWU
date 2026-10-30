#!/bin/bash
# Gazebo 배틀 경기장에서 미션 2 한 판 (PC 전용, 도메인 60~79 — 미션 1 시뮬레이션 80~99·실물 30과 분리).
#   tools/sim/run_battle.sh SEED [제한 s=600] [gui]
#   결과 한 줄: seed, 심판 판정(끝난 이유, 300 s 시점 보유자, 처음 잡은 시각, 우리 누적/마지막 연속 보유, 탈락한 상대,
#   우리 로봇이 모서리 칸에 가장 가까이 간 거리)
#   로그·world: ${SIM_OUT:-/tmp/cwu_sim}/battle_SEED.{log,world}
cd "$(dirname "$0")/../.." || exit 1
seed=$1 limit=${2:-600} gui=false out=${SIM_OUT:-/tmp/cwu_sim}
[ "$3" = gui ] && gui=true
mkdir -p "$out"
export ROS_DOMAIN_ID=$((60 + seed % 20)) GAZEBO_MODEL_DATABASE_URI=
export GAZEBO_MASTER_URI=http://localhost:$((11345 + ROS_DOMAIN_ID))
source /opt/ros/humble/setup.bash
source install/setup.bash
start=$(ros2 run cwu_sim make_battle --seed "$seed" --out "$out/battle_$seed.world") \
  || { echo "seed $seed | world 생성 실패"; exit 1; }
setsid ros2 launch cwu_sim battle.launch.py world:="$out/battle_$seed.world" start:="$start" seed:="$seed" gui:=$gui \
  > "$out/battle_$seed.log" 2>&1 &
group=$!
for _ in $(seq "$limit"); do
  sleep 1
  grep -q '심판 종료' "$out/battle_$seed.log" && break
  kill -0 $group 2> /dev/null || break
done
kill -INT -- -$group 2> /dev/null; sleep 5; kill -9 -- -$group 2> /dev/null
end=$(grep -o '심판 종료: .*' "$out/battle_$seed.log" | tail -1)
echo "seed $seed | ${end:-시간 초과/중단}"
