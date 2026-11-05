#!/usr/bin/env bash
# 启动容器中的 RViz，并自动刷新宿主机 X11/Xwayland 授权。
#
# 用法：
#   ./rviz.sh
#   ./rviz.sh -d /workspace/src/odom_ws/path/to/config.rviz
#   ./rviz.sh --odom-calib       # 加载里程计标定作业的预设显示配置
#
# 不要直接使用 `docker exec ... rviz`；那样会绕过 run.sh 中每次执行时
# 刷新的 DISPLAY 和 XAUTHORITY。run.sh 会 source x11_setup.sh，并把当前
# 图形会话可用的 cookie 传给 docker exec。若 /tmp/.docker.xauth 曾被误建为
# 目录，请先运行：sudo rmdir /tmp/.docker.xauth；否则旧容器挂载的是该目录，
# 需要执行 ./run.sh --recreate 后才能挂载授权文件。
set -euo pipefail

cd "$(dirname "$0")"

if [ "${1:-}" = "--odom-calib" ]; then
  shift
  set -- -d /workspace/src/odom_ws/src/calib_odom/rviz/odom_calib.rviz "$@"
fi

# 将每个 RViz 参数转义后组成一条交给容器内 bash -lc 执行的命令，既支持
# `-d <配置路径>` 等参数，也避免路径中存在空格时被错误拆分。
command=(rviz "$@")
printf -v command_line '%q ' "${command[@]}"

exec ./run.sh "$command_line"
