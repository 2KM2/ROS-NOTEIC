#!/usr/bin/env bash
# 把镜像导出成归档文件,用于离线恢复 / 迁移到别的机器。
#
#   ./save_image.sh                       -> 存到 ~/docker-images/
#   OUT_DIR=/mnt/backup ./save_image.sh   -> 换目录
#
# 注意:归档只是"保险",可重建的来源始终是 Dockerfile。平时在容器里 apt install 了
# 什么,请加进 Dockerfile 重新 build,不要用 docker commit。
set -euo pipefail
cd "$(dirname "$0")"

IMAGE=ros_noetic_hw:latest
OUT_DIR="${OUT_DIR:-$HOME/docker-images}"
OUT="$OUT_DIR/ros_noetic_hw.tar.gz"

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "镜像 $IMAGE 不存在,先执行 ./build_image.sh" >&2
  exit 1
fi

mkdir -p "$OUT_DIR"

AVAIL_KB=$(df -Pk "$OUT_DIR" | awk 'NR==2 {print $4}')
if [ "$AVAIL_KB" -lt 5000000 ]; then
  echo "警告: $OUT_DIR 可用空间只剩 $((AVAIL_KB / 1024)) MB,导出可能失败" >&2
fi

echo "导出 $IMAGE -> $OUT (几分钟,请等待)"
docker save "$IMAGE" | gzip -1 > "$OUT.partial"
mv "$OUT.partial" "$OUT"

echo
echo "完成: $OUT ($(du -h "$OUT" | cut -f1))"
echo "恢复: ./load_image.sh $OUT"
