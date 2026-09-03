# 装到容器的 /etc/profile.d/10-ros-env.sh
# 对 login shell(含 `bash -lc "cmd"` 这种非交互的)和交互 shell 都生效。

# Qt 需要 XDG_RUNTIME_DIR,否则会刷 "QStandardPaths: XDG_RUNTIME_DIR not set"
if [ -z "${XDG_RUNTIME_DIR:-}" ] || [ ! -w "${XDG_RUNTIME_DIR}" ]; then
    XDG_RUNTIME_DIR="/tmp/runtime-$(id -un)"
    export XDG_RUNTIME_DIR
    mkdir -p "$XDG_RUNTIME_DIR" 2>/dev/null || true
    chmod 700 "$XDG_RUNTIME_DIR" 2>/dev/null || true
fi

# catkin 工作空间根目录。run.sh 会用 -e ROS_WORKSPACE=... 传进来(见 ws_config.sh)。
: "${ROS_WORKSPACE:=/workspace}"
export ROS_WORKSPACE

if [ -f /opt/ros/noetic/setup.bash ]; then
    # setup.bash 里有未定义变量的引用,临时关掉 nounset 以防调用方开了 set -u
    _ros_had_u=0
    case "$-" in *u*) _ros_had_u=1; set +u ;; esac

    . /opt/ros/noetic/setup.bash
    [ -f "${ROS_WORKSPACE}/devel/setup.bash" ] && . "${ROS_WORKSPACE}/devel/setup.bash"

    [ "$_ros_had_u" = "1" ] && set -u
    unset _ros_had_u
fi
