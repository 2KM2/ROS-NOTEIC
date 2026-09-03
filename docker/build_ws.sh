#!/usr/bin/env bash
# 一键在容器里编译工作空间(不进入交互 shell)
#   ./build_ws.sh          -> catkin_make
#   ./build_ws.sh clean    -> 先清掉 build/devel/install 再编译
#
# 工作空间路径由 ws_config.sh 决定(默认 /workspace,即仓库根)。
# 临时换成 src/hw_1 作为根:  WS_SUBDIR=src/hw_1 ./build_ws.sh
set -euo pipefail
cd "$(dirname "$0")"

# shellcheck source=ws_config.sh
source ./ws_config.sh

echo "[build] 工作空间: $WS"

if [ "${1:-}" = "clean" ]; then
  ./run.sh "rm -rf '$WS/build' '$WS/devel' '$WS/install'"
fi

./run.sh "cd '$WS' && catkin_make -j\$(nproc)"
