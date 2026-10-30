#!/bin/bash
# PC에서 미션 1 로그 실시간 보기. Ctrl+C는 보기만 종료하고 미션은 계속 실행합니다.
#   tools/field/logs.sh            # 전체
#   tools/field/logs.sh 격자       # 그 글자가 든 줄만 (예: 격자 보정 = 위치 오차, →  = 상태 전이, 막힘)
if [ -n "$1" ]; then
  exec ssh rpi "tail -n 1000 -F ~/mission1.log | grep --line-buffered -- '$1'"
fi
exec ssh rpi 'tail -n 100 -F ~/mission1.log'
