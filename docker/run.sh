#!/usr/bin/env bash
# 启动/进入容器(带 X11 转发,rviz 可直接显示)
#   ./run.sh                      -> 交互 shell
#   ./run.sh "catkin_make"        -> 在容器里跑一条命令
set -euo pipefail
cd "$(dirname "$0")"

REPO_ROOT="$(cd .. && pwd)"
IMAGE=ros_noetic_hw:latest
CONTAINER=ros_noetic_hw

# --- 工作空间路径(容器内) ---
# shellcheck source=ws_config.sh
source ./ws_config.sh

# --- 准备 DISPLAY 和 X11 授权(兼容 Xorg / Wayland+Xwayland / SSH 转发) ---
# shellcheck source=x11_setup.sh
source ./x11_setup.sh
XAUTH="${XAUTH_FILE:-/tmp/.docker.xauth}"

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "镜像 $IMAGE 不存在,先执行 ./build_image.sh" >&2
  exit 1
fi

# 非终端环境(CI / 管道)下不要 -it
TTY_ARGS=(-i)
if [ -t 0 ] && [ -t 1 ]; then
  TTY_ARGS=(-it)
fi

# --- 容器已在运行 -> exec 进去 ---
if [ "$(docker ps -q -f name="^${CONTAINER}$")" ]; then
  RUNNING_IMG=$(docker inspect -f '{{.Image}}' "$CONTAINER")
  CURRENT_IMG=$(docker image inspect -f '{{.Id}}' "$IMAGE")
  if [ "$RUNNING_IMG" != "$CURRENT_IMG" ]; then
    echo "[warn] 容器 $CONTAINER 正在运行的是旧镜像,镜像已更新过。" >&2
    echo "[warn] 想用新镜像:先退出该容器(或 docker rm -f $CONTAINER)再执行 ./run.sh" >&2
  fi
  exec docker exec "${TTY_ARGS[@]}" \
    -e DISPLAY="$DISPLAY" \
    -e XAUTHORITY="$XAUTH" \
    -e ROS_HOSTNAME=localhost \
    -e ROS_MASTER_URI=http://localhost:11311 \
    -e ROS_WORKSPACE="$WS" \
    -w "$WS" \
    "$CONTAINER" bash -lc "${*:-bash}"
fi

# 有停止的同名容器 -> 删掉重建,保证挂载/环境一致
if [ "$(docker ps -aq -f name="^${CONTAINER}$")" ]; then
  docker rm -f "$CONTAINER" >/dev/null
fi

GPU_ARGS=()
if [ -d /dev/dri ]; then
  GPU_ARGS+=(--device /dev/dri)
fi

# 注意: --network host 下不要用 --hostname 改主机名。容器用的是宿主机的 /etc/hosts,
# 自定义主机名解析不了,roscore 会卡在名字解析上起不来。统一用 localhost。
exec docker run "${TTY_ARGS[@]}" --rm \
  --name "$CONTAINER" \
  --network host \
  --ipc host \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY="$XAUTH" \
  -e ROS_HOSTNAME=localhost \
  -e ROS_MASTER_URI=http://localhost:11311 \
  -e ROS_WORKSPACE="$WS" \
  -e LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-0}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$XAUTH:$XAUTH:rw" \
  -v "$REPO_ROOT:/workspace:rw" \
  "${GPU_ARGS[@]}" \
  -w "$WS" \
  "$IMAGE" \
  bash -lc "${*:-bash}"
