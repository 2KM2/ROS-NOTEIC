# tmux 教程（给这个 ROS Docker 环境用）

镜像里已经装了 tmux。容器是常驻的：`./run.sh` 退出只关你这个 shell，**tmux 会话还在容器里跑**。
所以正确用法是：进容器 → 开一个 tmux → 里面切窗口跑 `roslaunch` / 键盘 / `catkin_make`，断开后下次还能接回去。

## 为什么要用

ROS 经常要同时开好几个终端，例如：

```text
窗口 1:  roslaunch grid_search_practice practice.launch
窗口 2:  rosrun grid_search_practice practice_keyboard
窗口 3:  catkin_make / 看 rostopic
```

不用 tmux 就得反复 `./run.sh` 开多个 docker exec。用 tmux：进一次容器，里面随便切。

## 30 秒上手

```bash
cd docker && ./run.sh          # 进容器
tmux new -s ros                # 新建名为 ros 的会话
```

之后只记 **前缀键 `Ctrl-b`**：先按 `Ctrl-b`，松开，再按下一个键。

| 你想做的事 | 按键 |
|---|---|
| 新开一个窗口 | `Ctrl-b` `c` |
| 下一个 / 上一个窗口 | `Ctrl-b` `n` / `Ctrl-b` `p` |
| 跳到窗口 0、1、2… | `Ctrl-b` `0` … `Ctrl-b` `9` |
| 左右分屏 | `Ctrl-b` `%` |
| 上下分屏 | `Ctrl-b` `"` |
| 在分屏之间跳 | `Ctrl-b` 方向键 |
| 关掉当前窗格 | 输入 `exit`，或 `Ctrl-b` `x` 再 `y` |
| 暂时离开（会话继续跑） | `Ctrl-b` `d` |
| 下次回来接着用 | `tmux attach -t ros` |
| 看有哪些会话 | `tmux ls` |

底栏会显示窗口列表。`*` 是当前窗口。

## 配合本仓库的推荐布局

进容器后：

```bash
tmux new -s ros
```

1. 当前窗口跑 launch（会带起 RViz）：

   ```bash
   source /workspace/devel/setup.bash
   roslaunch grid_search_practice practice.launch
   ```

2. `Ctrl-b` `c` 开第二个窗口，跑键盘：

   ```bash
   source /workspace/devel/setup.bash
   rosrun grid_search_practice practice_keyboard
   ```

3. 需要编译再 `Ctrl-b` `c`：

   ```bash
   cd /workspace && catkin_make -j$(nproc)
   ```

`Ctrl-b` `d` 断开。宿主机终端关掉也没关系，容器里的 roslaunch 还在。下次：

```bash
cd docker && ./run.sh
tmux attach -t ros
```

## 会话、窗口、窗格

```text
session（会话）ros
 ├─ window 0: launch
 │    ├─ pane 左: roslaunch
 │    └─ pane 右: 看 rostopic hz
 ├─ window 1: keyboard
 └─ window 2: build
```

- **会话**：一整组终端，用名字记住。断开不等于结束。
- **窗口**：类似浏览器标签。`Ctrl-b` `c` 新建，`Ctrl-b` `,` 改名。
- **窗格**：一个窗口里再左右/上下切。适合一边跑节点一边盯 topic。

常用会话命令（在普通 shell 里敲，不需要先按 `Ctrl-b`）：

```bash
tmux new -s ros              # 创建
tmux attach -t ros           # 接上；已经在里面会报错，先 Ctrl-b d
tmux ls                      # 列表
tmux kill-session -t ros     # 整会话杀掉（里面的 roslaunch 也会停）
tmux kill-server             # 杀掉容器里所有 tmux
```

会话已存在还 `new -s ros` 会失败。第一次用 `new`，之后一律 `attach`。偷懒可以：

```bash
tmux new -As ros             # 没有就建，有就接上
```

## 分屏再记几下

| 按键 | 作用 |
|---|---|
| `Ctrl-b` `%` | 左右切 |
| `Ctrl-b` `"` | 上下切 |
| `Ctrl-b` `方向键` | 换焦点 |
| `Ctrl-b` `Ctrl-方向键` | 调窗格大小 |
| `Ctrl-b` `{` / `}` | 窗格对调位置 |
| `Ctrl-b` `z` | 当前窗格最大化 / 还原 |
| `Ctrl-b` `x` | 关掉当前窗格（确认 y） |
| `Ctrl-b` `空格` | 轮换分屏布局 |

## 复制、滚动、搜日志

节点刷屏时：

1. `Ctrl-b` `[` 进入复制模式（能用方向键 / PageUp 往上翻）
2. 空格开始选，空格结束（默认 vi 风格）
3. `q` 退出复制模式
4. `Ctrl-b` `]` 粘贴

搜：复制模式里按 `/`，输入关键字，`n` 下一个。

## 和这个 Docker 环境有关的坑

1. **`exit` 掉 `./run.sh` 不会停 tmux。** 这是故意的。要停 ROS 节点：先 `attach` 进去 `Ctrl-c`，或 `tmux kill-session -t ros`。
2. **`./run.sh --recreate` 或 `docker rm -f ros_noetic_hw` 会丢掉所有 tmux 会话。** 源码和 `build/` 还在，只是会话没了。
3. **每个新窗口都要 `source /workspace/devel/setup.bash`。** tmux 新开的 pane/window 不一定带上你当前的环境。`catkin_make` 之后旧窗口也要重新 source 一次。
4. **键盘节点要独占终端。** `practice_keyboard` 会把终端设成 raw 模式，放在单独窗口，不要和 roslaunch 挤在同一个窗格。
5. **前缀键是先松开再按第二个键。** 按住 `Ctrl-b` 不放再狂按 `c` 经常没反应。
6. **嵌套 tmux**（宿主机也开了 tmux 再进容器开一层）：内层要按两次前缀，`Ctrl-b` `Ctrl-b` `c`。能不用就不用。

## 最小配置（可选）

容器里的 `~/.tmux.conf` 不随源码走，`--recreate` 会丢。真要定制，写进 Dockerfile 或自己每次 attach 后 source。入门不必改。若鼠标滚轮想翻历史，可临时：

```bash
tmux set -g mouse on
```

## 一张图记住

```text
Ctrl-b 然后：

  d     断开（节点继续跑）
  c     新窗口
  n p   切窗口
  0-9   跳窗口
  % "   分屏
  方向  换窗格
  z     窗格最大化
  [     翻日志
  x     关窗格
```

对照练习：`grid_search_practice` 的 README 就是「一个 launch + 一个 keyboard」，用上面两个窗口即可。
