FROM osrf/ros:noetic-desktop-full

# ---- 可选:国内 apt 镜像加速。构建时加 --build-arg USE_CN_MIRROR=1 启用 ----
ARG USE_CN_MIRROR=0
RUN if [ "$USE_CN_MIRROR" = "1" ]; then \
      sed -i 's|http://archive.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g; \
              s|http://security.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g' \
              /etc/apt/sources.list ; \
    fi

ENV DEBIAN_FRONTEND=noninteractive

# 编译 hw_1 所需依赖:
#   grid_path_searcher -> roscpp std_msgs nav_msgs visualization_msgs sensor_msgs, Eigen3, PCL, pcl_conversions
#   waypoint_generator -> roscpp tf nav_msgs
#   rviz_plugins       -> rviz, Qt5 (Core/Widgets)
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        vim \
        nano \
        less \
        wget \
        curl \
        gdb \
        sudo \
        python3-pip \
        python3-catkin-tools \
        python3-rosdep \
        python3-osrf-pycommon \
        libeigen3-dev \
        libpcl-dev \
        libdw-dev \
        binutils-dev \
        qtbase5-dev \
        qt5-default \
        ros-noetic-rviz \
        ros-noetic-pcl-ros \
        ros-noetic-pcl-conversions \
        ros-noetic-tf \
        ros-noetic-nav-msgs \
        ros-noetic-visualization-msgs \
        ros-noetic-sensor-msgs \
        ros-noetic-rqt \
        ros-noetic-rqt-common-plugins \
    && rm -rf /var/lib/apt/lists/*

# 顺手工具 + GUI/X11 调试:xdpyinfo、xset、xhost、glxinfo、xeyes
# 在容器里临时 apt install 的东西一律加到这里,别用 docker commit —— 那样镜像就跟
# Dockerfile 对不上了,重建一次就丢。
RUN apt-get update && apt-get install -y --no-install-recommends \
        tmux \
        x11-utils \
        x11-xserver-utils \
        x11-apps \
        mesa-utils \
        libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# ---- 创建与宿主机同 UID/GID 的用户,避免挂载目录产生 root 属主的 build/devel ----
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
RUN if getent group ${USER_GID} >/dev/null; then \
        EXIST_GRP=$(getent group ${USER_GID} | cut -d: -f1); \
        echo "reuse group ${EXIST_GRP}"; \
    else \
        groupadd -g ${USER_GID} ${USERNAME}; \
    fi \
    && if getent passwd ${USER_UID} >/dev/null; then \
        userdel -r "$(getent passwd ${USER_UID} | cut -d: -f1)" 2>/dev/null || true; \
    fi \
    && useradd -m -s /bin/bash -u ${USER_UID} -g ${USER_GID} ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# rosdep 数据库(root 与普通用户都可用)
RUN rosdep update --rosdistro noetic || true
USER ${USERNAME}
RUN rosdep update --rosdistro noetic || true
USER root

# 自动 source ROS 环境 + 工作空间 devel。
# 放在 /etc/profile.d 而不是只放 .bashrc:`docker exec ... bash -lc "cmd"` 是非交互
# login shell,.bashrc 会因为 `case $- in *i*)` 提前 return,profile.d 则一定会执行。
COPY ros_env.sh /etc/profile.d/10-ros-env.sh
RUN chmod 644 /etc/profile.d/10-ros-env.sh \
    && echo 'source /etc/profile.d/10-ros-env.sh' >> /home/${USERNAME}/.bashrc

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

USER ${USERNAME}
WORKDIR /workspace/src/hw_1
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
