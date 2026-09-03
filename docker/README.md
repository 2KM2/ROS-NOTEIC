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
│   ├── run.sh               <- 进容器(常驻,exit 不删)/ 在容器里执行命令
│   └── build_ws.sh          <- 一键编译
└── src/                     <- source space
    └── hw_1/src/
        ├── grid_path_searcher
        ├── waypoint_generator
        ├── rviz_plugins
        └── multi_map_server   <- 补的消息包(见下文 rviz_plugins 一节)
```

catkin 会在 `src/` 下递归查找 `package.xml`,所以这些包嵌在 `src/hw_1/src/` 里也能正常发现
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

## 容器生命周期(常驻,exit 不会删)

容器本体跑的是 `sleep infinity`,`./run.sh` 的每个 shell 都是 `docker exec` 进去的。
所以 `exit` 只结束你这个 shell,容器和里面的 tmux / 后台节点继续在跑:

```bash
./run.sh                 # 不存在就创建;已停止就 start;在跑就直接 exec 进去
./run.sh --recreate      # 销毁重建(改了镜像或挂载后用)
docker stop ros_noetic_hw   # 手动停(下次 ./run.sh 自动 start)
docker rm -f ros_noetic_hw  # 彻底删
```

`--restart unless-stopped`:开机 / docker 重启后容器自动回来,除非你自己 `docker stop` 过。

### 什么会丢、什么不会丢

| 东西 | 位置 | `exit` 后 | 容器被删后 |
|---|---|---|---|
| 源码、`build/`、`devel/` | 宿主机(`../:/workspace` 是 bind mount) | 不丢 | **不丢** |
| 镜像 `ros_noetic_hw:latest` | 本地 docker | 不丢 | **不丢** |
| tmux session、后台 roscore | 容器内存 | **不丢** | 会丢 |
| 在容器里 `apt install` 的包 | 容器可写层 | 不丢 | **会丢** |
| `~/.ros/log`、mesa shader 缓存 | 容器可写层 | 不丢 | 会丢(无所谓) |

所以**唯一需要操心的是临时装的包**。做法是加进 Dockerfile 然后 `./build_image.sh`,
**不要用 `docker commit`** —— commit 出来的层无法从 Dockerfile 重建,下次 build 就丢了,
镜像和 Dockerfile 也会越走越远。想知道容器里比镜像多了什么:

```bash
docker diff ros_noetic_hw | grep -E 'usr/(bin|lib)|var/lib/dpkg/info'
```

镜像改过之后,已有容器不会自动更新;`run.sh` 会打 `[warn] 容器用的是旧镜像`
提醒你,执行 `./run.sh --recreate` 换过去。

### 离线备份 / 迁移到别的机器(可选)

```bash
./save_image.sh                       # -> ~/docker-images/ros_noetic_hw.tar.gz
OUT_DIR=/mnt/usb ./save_image.sh      # 换目录
./load_image.sh [归档路径]            # 恢复
```

平时不需要 —— Dockerfile 已入库,`./build_image.sh` 随时能重建(base 镜像在本地的话约 2 分钟)。
归档只在断网、重装、或换一台没网的机器时有用。

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

### 6. 常驻容器 + 单文件 bind mount 的 inode 陷阱

`-v /tmp/.docker.xauth:/tmp/.docker.xauth` 绑的是 **inode**,不是路径。
`x11_setup.sh` 每次运行都会重写这个文件,而 `rm + touch`、以及 `xauth nmerge`
自己的 write-then-rename,都会换掉 inode —— 容器创建时绑的老 inode 还在,
容器里看到的就永远是那份旧内容(重启宿主机后就是个空文件)→ rviz 又报
`Authorization required`。

修法:cookie 先写到临时文件,最后用 `cat tmp > /tmp/.docker.xauth` **原地覆写**。
`DISPLAY`/`XAUTHORITY` 则在每次 `docker exec` 时重新 `-e` 传进去,
所以宿主机换了图形会话也不用重建容器。

## rviz_plugins 的三个 Display —— 已补齐编译

原来 `plugin_description.xml` 声明了 `ProbMapDisplay` / `AerialMapDisplay` /
`MultiProbMapDisplay`,但 `CMakeLists.txt` 里对应的 `.cpp` 和 moc 头全被注释掉了,
加载 `rviz_plugins/config/rviz_config.rviz` 会报三条
`PluginlibFactory: The plugin for class '...' failed to load.`

现在四个类都真编进 `librviz_plugins.so` 了。改动:

1. `rviz_plugins/CMakeLists.txt`:三个 `.cpp` 放回 `SOURCE_FILES`、三个 `.h` 放回
   `qt5_wrap_cpp`;`find_package` 补 `multi_map_server nav_msgs tf pluginlib`;
   `target_link_libraries` 补 `${catkin_LIBRARIES}`;加
   `add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})`
   —— 否则 clean build 时消息头还没生成就开始编本包。
2. **新建 `src/hw_1/src/multi_map_server`**:`multi_probmap_display.h` 要
   `multi_map_server/MultiOccupancyGrid.h`,而这个包作业里没给、Noetic 的 apt 源里也没有。
   按 `multi_probmap_display.cpp` 的字段用法还原成一个只含消息的最小包:

   ```
   nav_msgs/OccupancyGrid[] maps
   geometry_msgs/Pose[] origins
   ```

3. `install(DIRECTORY media/ icons/)` 指向的目录本包里不存在,`catkin_make install`
   会直接 CMake Error。加了 `if(EXISTS ...)` 判断。

验证:clean 全量编译零 error;`nm -DC devel/lib/librviz_plugins.so` 四个
`registerPlugin<...>` 都在;`rviz -d rviz_plugins/config/rviz_config.rviz` 零报错,
`/proc/<pid>/maps` 确认加载的是 `devel/lib/librviz_plugins.so`;`catkin_make install` 通过。

作业原来自带的预编译 `src/hw_1/src/rviz_plugins/lib/librviz_plugins.so` 已删除
—— `plugin_description.xml` 里的 `<library path="lib/librviz_plugins">` 是 pluginlib
的标准相对路径,解析的是 catkin 的 lib 目录(`devel/lib`),不是包源码目录,
所以那个文件从来没被加载过。
