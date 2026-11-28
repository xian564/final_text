#!/bin/bash
# 抓 RViz 窗口截图（X11）
# 用法: ./scripts/capture_rviz.sh /tmp/rviz.png
set -e
export DISPLAY="${DISPLAY:-:0}"

if ! command -v xwininfo >/dev/null || ! command -v import >/dev/null; then
  echo "缺少工具，先装: sudo apt install -y x11-utils imagemagick"
  exit 1
fi

ID=$(xwininfo -root -tree 2>/dev/null | grep -i rviz | head -1 | awk '{print $1}')
if [ -z "$ID" ]; then
  echo "没找到 RViz 窗口；先 roslaunch 起 RViz"
  exit 1
fi

OUT="${1:-/tmp/rviz.png}"
import -window "$ID" "$OUT"
echo "saved $OUT (window $ID)"
