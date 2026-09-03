#!/usr/bin/env bash
# 从 save_image.sh 导出的归档恢复镜像
#   ./load_image.sh [归档路径]     默认 ~/docker-images/ros_noetic_hw.tar.gz
set -euo pipefail

ARCHIVE="${1:-$HOME/docker-images/ros_noetic_hw.tar.gz}"

if [ ! -f "$ARCHIVE" ]; then
  echo "找不到归档: $ARCHIVE" >&2
  exit 1
fi

echo "从 $ARCHIVE 恢复镜像..."
gunzip -c "$ARCHIVE" | docker load

echo
docker images ros_noetic_hw
echo "进入容器: ./run.sh"
