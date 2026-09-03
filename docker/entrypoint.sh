#!/usr/bin/env bash
set -e

# 环境设置集中在 /etc/profile.d/10-ros-env.sh(ros_env.sh),
# 这样 `docker run` 走 entrypoint、`docker exec ... bash -lc` 走 profile,两条路径一致。
# shellcheck source=ros_env.sh
source /etc/profile.d/10-ros-env.sh

exec "$@"
