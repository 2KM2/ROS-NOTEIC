# ROS Catkin 同名包处理说明

## 问题原因

在同一个 Catkin 工作空间中，ROS 包名必须唯一。当前两个作业工程都包含以下同名包：

- `grid_path_searcher`
- `rviz_plugins`
- `waypoint_generator`

虽然它们位于不同目录，例如 `hw_1/src` 和 `hw_2/src`，但 Catkin 只根据 `package.xml` 中的 `<name>` 判断包名，因此会报错：

```text
Multiple packages found with the same name
```

两个作业的源代码可以同时保留，但不能让它们同时在同一个 Catkin 构建空间中参与构建。

## 推荐方案：分别构建两个作业

在 `/workspace` 下分别为两个作业指定 source、build、devel 和 install 空间：

```bash
cd /workspace

catkin_make \
  --source-space src/hw_1/src \
  --build-space build_hw1 \
  --devel-space devel_hw1 \
  --install-space install_hw1

catkin_make \
  --source-space src/hw_2/src \
  --build-space build_hw2 \
  --devel-space devel_hw2 \
  --install-space install_hw2
```

构建完成后，会得到两套互相独立的环境：

```text
/workspace/build_hw1   /workspace/devel_hw1   /workspace/install_hw1
/workspace/build_hw2   /workspace/devel_hw2   /workspace/install_hw2
```

## 运行 hw_1

```bash
source /workspace/devel_hw1/setup.bash
roslaunch grid_path_searcher demo.launch
```

## 运行 hw_2

```bash
source /workspace/devel_hw2/setup.bash
roslaunch grid_path_searcher demo.launch
```

建议分别在不同终端中运行两个作业。不要在同一个终端里连续 source 两套 `setup.bash`，否则同名包可能互相覆盖。

## 临时方案：使用 `CATKIN_IGNORE`

如果只需要暂时编译其中一个作业，可以屏蔽另一个作业的 source 子目录。例如屏蔽 hw_1：

```bash
cd /workspace
touch src/hw_1/src/CATKIN_IGNORE
catkin_make
```

恢复 hw_1：

```bash
rm src/hw_1/src/CATKIN_IGNORE
```

切换到只编译 hw_2 时，可以对 hw_2 使用同样的方法：

```bash
touch src/hw_2/src/CATKIN_IGNORE
```

注意：`CATKIN_IGNORE` 会忽略对应目录下的全部 ROS 包。当前 hw_1 还包含 `multi_map_server`，因此使用该方法时它也会被一起忽略。

## 不推荐方案：直接重命名包

也可以把其中一套包改名，例如改成 `hw2_grid_path_searcher`，但必须同步修改：

- `package.xml` 中的 `<name>`
- `CMakeLists.txt`
- launch 文件中的 `pkg="..."`
- `ros::package`、插件配置和其他依赖引用

这会改变原作业工程的包接口，除非确实需要同时运行两套同名节点，否则不建议采用。

## 结论

保留两个作业工程的源代码，并分别构建 `hw_1` 和 `hw_2`，是最稳定、改动最少的处理方式。
