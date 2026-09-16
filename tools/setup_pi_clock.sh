#!/bin/bash
# 라즈베리파이 시계를 HTTP 헤더 기반으로 자동 보정하도록 설정합니다. 1회만 실행하면 됩니다.
#
# 왜 필요한가: 파이 4에는 RTC가 없어 전원을 내리면 시계가 과거로 되돌아갑니다.
# 이 현장 네트워크는 UDP 123(NTP)이 공유기·외부 모두 차단돼 systemd-timesyncd가
# 영구히 실패합니다("System clock synchronized: no"). 시계가 틀리면 ROS 메시지
# 타임스탬프가 어긋나 TF 조회가 전부 실패하고, HTTPS 인증서도 "아직 유효하지 않음"으로
# 거부돼 git/apt가 막힙니다.
#
# htpdate는 HTTP 응답의 Date 헤더에서 시각을 읽습니다. TCP 80/443은 열려 있으므로
# NTP가 막힌 환경에서도 동작합니다.
#
# 사용법:  ssh -t rpi 'sudo bash ~/ros_CWU/tools/setup_pi_clock.sh'
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "root 권한이 필요합니다: sudo bash $0" >&2
  exit 1
fi

# 시각이 크게 틀어져 있으면 apt의 Release 파일 유효성 검사가 먼저 실패합니다.
# 설치 전에 한 번 수동으로 맞춰 부트스트랩 문제를 없앱니다.
echo "== 부트스트랩: HTTP Date 헤더로 현재 시각 설정 =="
for host in https://www.google.com http://ports.ubuntu.com; do
  d=$(curl -sI --max-time 10 "$host" 2>/dev/null | grep -i '^date:' | cut -d' ' -f2-) || true
  if [ -n "${d:-}" ]; then
    date -u -s "$d" >/dev/null && echo "  $host 기준으로 설정: $(date -u)"
    break
  fi
done

echo "== htpdate 설치 =="
apt-get update -qq
apt-get install -y htpdate

echo "== 설정 =="
# -D 데몬, -s 시각 설정(점진 조정 -a는 큰 격차를 따라잡지 못함),
# -t 안전장치 해제(기본값은 큰 폭의 점프를 거부해 부팅 직후 보정이 안 됨),
# -4 IPv4 강제(timesyncd가 경로 없는 IPv6 주소를 골라 타임아웃되던 문제 회피).
sed -i 's|^HTP_SERVERS=.*|HTP_SERVERS="www.google.com github.com ports.ubuntu.com"|' /etc/default/htpdate
sed -i 's|^HTP_OPTIONS=.*|HTP_OPTIONS="-D -s -t -4"|' /etc/default/htpdate
grep -E '^HTP_(SERVERS|OPTIONS)=' /etc/default/htpdate

# timesyncd를 남겨두면 도달 불가능한 NTP 서버를 계속 폴링하며 로그를 채웁니다.
systemctl disable --now systemd-timesyncd 2>/dev/null || true
timedatectl set-ntp false 2>/dev/null || true

systemctl enable htpdate
systemctl restart htpdate
sleep 3
systemctl --no-pager --lines=0 status htpdate | head -4
echo "== 완료: $(date -u) (UTC) / $(date) =="
