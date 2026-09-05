#!/usr/bin/env bash
# 构建镜像。用法:
#   ./build_image.sh              # 官方源
#   USE_CN_MIRROR=1 ./build_image.sh   # 阿里云 apt 镜像加速
#   BASE_IMAGE=docker.m.daocloud.io/osrf/ros:noetic-desktop-full ./build_image.sh
set -euo pipefail
cd "$(dirname "$0")"

docker build \
  --build-arg BASE_IMAGE="${BASE_IMAGE:-osrf/ros:noetic-desktop-full}" \
  --build-arg USE_CN_MIRROR="${USE_CN_MIRROR:-0}" \
  --build-arg USER_UID="$(id -u)" \
  --build-arg USER_GID="$(id -g)" \
  -t ros_noetic_hw:latest \
  -f Dockerfile .

echo
echo "镜像已构建: ros_noetic_hw:latest"
echo "进入容器: ./run.sh"
