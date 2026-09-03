# ROS Noetic Docker 环境(编译 / 运行 hw_1)

## 目录结构

```
ROS_NOETIC/                  <- 仓库根,挂载为容器内 /workspace,同时也是 catkin 工作空间根
├── build/  devel/           <- catkin_make 产物(已 gitignore)
├── docker/
│   ├── Dockerfile
│   ├── ros_env.sh           <- 装到容器 /etc/profile.d/10-ros-env.sh
│   ├── entrypoint.sh
│   ├── ws_config.sh         <- 工作空间路径的唯一配置点
│   ├── x11_setup.sh         <- 生成容器可用的 X11 cookie
│   ├── docker-compose.yml
│   ├── build_image.sh       <- 构建镜像
│   ├── run.sh               <- 进容器 / 在容器里执行命令
│   └── build_ws.sh          <- 一键编译
└── src/                     <- source space
    └── hw_1/src/
        ├── grid_path_searcher
        ├── waypoint_generator
        └── rviz_plugins
```

catkin 会在 `src/` 下递归查找 `package.xml`,所以三个包嵌在 `src/hw_1/src/` 里也能正常发现
(`rospack find grid_path_searcher` → `/workspace/src/hw_1/src/grid_path_searcher`)。

## 快速开始

```bash
cd docker

# 1) 构建镜像(基于 osrf/ros:noetic-desktop-full)
./build_image.sh
#    国内网络加速 apt: USE_CN_MIRROR=1 ./build_image.sh

# 2) 编译
./build_ws.sh            # 增量
./build_ws.sh clean      # 清 build/devel/install 后全量

# 3) 进容器(已自动 source ROS + devel/setup.bash)
./run.sh

# 或直接在容器里跑单条命令
./run.sh "rospack find grid_path_searcher"
./run.sh "catkin_make -j8"
```

## 运行 demo

`demo.launch` 只起 3 个节点,**不带 rviz**,需要另开一个终端手动起 rviz:

```bash
# 终端 1
cd docker && ./run.sh
roslaunch grid_path_searcher demo.launch

# 终端 2(会 exec 进同一个容器)
cd docker && ./run.sh
rviz -d $(rospack find grid_path_searcher)/launch/rviz_config/demo.rviz
```

然后在 rviz 里用 `3D Nav Goal` 工具点目标点。

## 换工作空间根目录

只改 `ws_config.sh` 一处,或临时用环境变量:

```bash
WS_SUBDIR=src/hw_1 ./build_ws.sh    # 改成以 src/hw_1 为工作空间根
```

## 镜像里装了什么

- 基础镜像 `osrf/ros:noetic-desktop-full`(含 rviz、PCL 1.10、pcl_conversions、tf)
- 编译工具:build-essential、cmake、python3-catkin-tools、rosdep、gdb
- X11/GL 调试:x11-utils(`xdpyinfo`)、x11-xserver-utils(`xset`/`xhost`)、mesa-utils(`glxinfo`)
- 容器内用户 `ros`,UID/GID 与宿主机当前用户一致(`build_image.sh` 自动传 `id -u`/`id -g`),
  所以挂载目录里生成的 `build/` `devel/` 属主是宿主机用户,不会变成 root。

## 踩过的坑(都已在脚本里处理)

### 1. PCL 1.10 需要 C++14 —— 已改源码

`grid_path_searcher/CMakeLists.txt` 原本写死 `-std=c++11`,在 Noetic 上报:

```
/usr/include/pcl-1.10/pcl/point_representation.h:252:48:
  error: the value of 'NrDims' is not usable in a constant expression
```

已改为 `-std=c++14`。这是 Noetic + PCL 1.10 的标准修法。

### 2. rviz 报 `Authorization required` / `could not connect to display :0`

宿主机是 **GNOME on Wayland**,`:0` 其实是 **Xwayland**,它的 MIT-MAGIC-COOKIE
**不在 `~/.Xauthority`**,而在 `/run/user/$UID/.mutter-Xwaylandauth.XXXXXX`
(文件名每次开机变,且旧文件会残留)。所以直接 `xauth nlist :0` 什么都取不到。

`x11_setup.sh` 的做法:
1. `DISPLAY` 为空时,从运行中的 `Xwayland`/`Xorg` 进程参数里读出 display 号;
2. 候选 auth 文件按优先级排:`$XAUTHORITY` → Xwayland 进程 `-auth` 指向的文件 →
   `~/.Xauthority` → `/run/user/$UID/.mutter-Xwaylandauth.*`;
3. **用 `xset q` 逐个验证**,只把真正能连上的那份 cookie 写进 `/tmp/.docker.xauth`
   (family 改成 `ffff`/FamilyWild,容器内才认)。
   过期的残留文件必须剔掉 —— 同一个 display 塞多条不同 cookie 会让 Xlib 用错。

实在不行的兜底(在宿主机图形会话终端里):`xhost +local:`

### 3. `roscore` 起不来,`rosnode list` 报 `Unable to communicate with master!`

`--network host` 下容器用的是**宿主机的 `/etc/hosts`**,而 `--hostname ros-noetic`
设的主机名在里面查不到 → `roscore` 卡在名字解析上、无任何输出。

修法:去掉 `--hostname`,并显式设 `ROS_HOSTNAME=localhost` +
`ROS_MASTER_URI=http://localhost:11311`。

### 4. `docker exec` 进已有容器时 ROS 环境没生效

`docker exec` 会跳过 `ENTRYPOINT`,而 `~/.bashrc` 开头有
`case $- in *i*) ;; *) return;; esac`,对 `bash -lc "cmd"` 这种非交互 shell 直接 return。

修法:环境设置放到 `/etc/profile.d/10-ros-env.sh`(login shell 一定会执行),
`entrypoint.sh` 和 `.bashrc` 都只是 source 它,两条路径行为一致。

### 5. `QStandardPaths: XDG_RUNTIME_DIR not set`

`ros_env.sh` 里自动建 `/tmp/runtime-$(id -un)` 并 `chmod 700`。

## 一个源码遗留问题(不影响作业,没动)

`rviz_plugins/plugin_description.xml` 声明了 `ProbMapDisplay` / `AerialMapDisplay` /
`MultiProbMapDisplay` 三个类,但 `CMakeLists.txt` 里对应的 `.cpp` 全被注释掉了没编译。
所以加载 `rviz_plugins/config/rviz_config.rviz` 时会报三条:

```
PluginlibFactory: The plugin for class 'rviz_plugins/ProbMapDisplay' failed to load.
```

作业实际用的 `grid_path_searcher/launch/rviz_config/demo.rviz` 不需要这三个 Display,
加载零报错。要消掉这些报错,把 plugin_description.xml 里那三个 `<class>` 删掉,
或在 CMakeLists 里把对应源文件放回 `SOURCE_FILES`。
