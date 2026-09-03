#!/usr/bin/env bash
# 进入容器(常驻式:退出 shell 不会删容器,tmux/后台进程都还在)
#
#   ./run.sh                  -> 交互 shell
#   ./run.sh "catkin_make"    -> 在容器里跑一条命令
#   ./run.sh --recreate       -> 销毁并重建容器(镜像更新后用这个)
#
# 容器本体跑的是 `sleep infinity`,所有 shell 都是 docker exec 进去的,
# 所以 exit 只结束你这个 shell,容器继续在跑。
#   停止: docker stop ros_noetic_hw     (下次 ./run.sh 会自动 start)
#   删除: docker rm -f ros_noetic_hw    (或 ./run.sh --recreate)
set -euo pipefail
cd "$(dirname "$0")"

REPO_ROOT="$(cd .. && pwd)"
IMAGE=ros_noetic_hw:latest
CONTAINER=ros_noetic_hw

RECREATE=0
if [ "${1:-}" = "--recreate" ] || [ "${1:-}" = "-r" ]; then
  RECREATE=1
  shift
fi

# shellcheck source=ws_config.sh
source ./ws_config.sh
# shellcheck source=x11_setup.sh
source ./x11_setup.sh
XAUTH="${XAUTH_FILE:-/tmp/.docker.xauth}"

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "镜像 $IMAGE 不存在,先执行 ./build_image.sh" >&2
  exit 1
fi

TTY_ARGS=(-i)
if [ -t 0 ] && [ -t 1 ]; then
  TTY_ARGS=(-it)
fi

exists()  { [ -n "$(docker ps -aq -f name="^${CONTAINER}$")" ]; }
running() { [ -n "$(docker ps  -q -f name="^${CONTAINER}$")" ]; }

if [ "$RECREATE" -eq 1 ] && exists; then
  echo "[run] 销毁旧容器 $CONTAINER"
  docker rm -f "$CONTAINER" >/dev/null
fi

# --- 容器不存在 -> 创建一个常驻容器 ---
if ! exists; then
  GPU_ARGS=()
  if [ -d /dev/dri ]; then
    GPU_ARGS+=(--device /dev/dri)
  fi

  # 注意: --network host 下不要用 --hostname。容器用宿主机的 /etc/hosts,
  # 自定义主机名解析不了,roscore 会卡在名字解析上起不来。
  echo "[run] 创建常驻容器 $CONTAINER"
  # --init: 让 tini 当 PID 1。否则 `sleep infinity` 是 PID 1,而 PID 1 默认忽略
  # SIGTERM,`docker stop` 要干等 10 秒超时才能 SIGKILL;顺带回收僵尸子进程。
  docker run -d --init \
    --name "$CONTAINER" \
    --restart unless-stopped \
    --network host \
    --ipc host \
    -e QT_X11_NO_MITSHM=1 \
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
    sleep infinity >/dev/null

elif ! running; then
  echo "[run] 启动已存在的容器 $CONTAINER"
  docker start "$CONTAINER" >/dev/null
fi

# --- 镜像更新过但容器还是老的 -> 提示 ---
RUNNING_IMG=$(docker inspect -f '{{.Image}}' "$CONTAINER")
CURRENT_IMG=$(docker image inspect -f '{{.Id}}' "$IMAGE")
if [ "$RUNNING_IMG" != "$CURRENT_IMG" ]; then
  echo "[warn] 容器用的是旧镜像。想用新镜像: ./run.sh --recreate" >&2
fi

# DISPLAY / XAUTHORITY 每次 exec 都重新传,宿主机图形会话换了也能跟上
exec docker exec "${TTY_ARGS[@]}" \
  -e DISPLAY="$DISPLAY" \
  -e XAUTHORITY="$XAUTH" \
  -e ROS_WORKSPACE="$WS" \
  -w "$WS" \
  "$CONTAINER" bash -lc "${*:-bash}"
