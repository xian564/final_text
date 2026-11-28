#!/bin/bash
# 抓最大的 RViz 窗口（排除弹窗/边框），可先关闭 ROS EOL 弹窗
# 用法: ./scripts/capture_rviz.sh /tmp/rviz.png
set -e
export DISPLAY="${DISPLAY:-:0}"
OUT="${1:-/tmp/rviz.png}"

# 关掉 "ROS 1 End-of-Life" 弹窗（若存在）
EOL=$(xdotool search --name "End-of-Life" 2>/dev/null | head -1 || true)
if [ -n "$EOL" ]; then
  xdotool windowactivate "$EOL" key Return 2>/dev/null || true
  sleep 0.3
fi

BEST=""; BEST_AREA=0
for id in $(xdotool search --class rviz 2>/dev/null); do
  info=$(xwininfo -id "$id" 2>/dev/null || true)
  w=$(echo "$info" | awk '/Width:/{print $2; exit}')
  h=$(echo "$info" | awk '/Height:/{print $2; exit}')
  [ -z "$w" ] && continue
  area=$((w*h))
  if [ "$area" -gt "$BEST_AREA" ]; then BEST_AREA=$area; BEST=$id; fi
done

if [ -z "$BEST" ]; then
  echo "没找到 RViz 窗口；先起 RViz"
  exit 1
fi

import -window "$BEST" "$OUT"
echo "saved $OUT (window $BEST, ${BEST_AREA}px)"
