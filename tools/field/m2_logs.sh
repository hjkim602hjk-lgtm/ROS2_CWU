#!/bin/bash
# PC에서 미션 2 로그 실시간 보기. Ctrl+C는 보기만 종료하고 미션은 계속 실행합니다.
#   tools/field/m2_logs.sh          # 전체
#   tools/field/m2_logs.sh →        # 그 글자가 든 줄만 (→ = 상태 전이, 보드 = 보드 명령·이벤트)
if [ -n "$1" ]; then
  exec ssh rpi "tail -n 1000 -F ~/mission2.log | grep --line-buffered -- '$1'"
fi
exec ssh rpi 'tail -n 100 -F ~/mission2.log'
