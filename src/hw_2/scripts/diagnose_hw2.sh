#!/bin/bash
# HW2 问题诊断脚本

echo "=========================================="
echo "HW2 诊断脚本"
echo "=========================================="

echo -e "\n[1/6] 检查所有运行的节点"
rosnode list
echo "预期看到: /demo_node, /random_complex, /waypoint_generator, /rviz"

echo -e "\n[2/6] 检查 random_complex 节点信息"
rosnode info /random_complex

echo -e "\n[3/6] 检查地图话题详情"
rostopic info /random_complex/global_map

echo -e "\n[4/6] 检查地图点云数据（不显示数组内容）"
timeout 3 rostopic echo /random_complex/global_map --noarr -n 1 || echo "超时或无数据"

echo -e "\n[5/6] 检查 demo_node 发布的可视化话题"
rostopic info /demo_node/grid_map_vis

echo -e "\n[6/6] 检查地图参数配置"
echo "--- random_complex 参数 ---"
rosparam list | grep random_complex
echo ""
echo "地图大小:"
rosparam get /random_complex/map/x_size
rosparam get /random_complex/map/y_size
rosparam get /random_complex/map/z_size
echo "障碍物数量:"
rosparam get /random_complex/map/obs_num
rosparam get /random_complex/map/circle_num

echo -e "\n=========================================="
echo "诊断建议:"
echo "=========================================="
echo "1. 如果 /random_complex/global_map 有数据但 rviz 是空的"
echo "   -> 原因: demo_node 需要收到目标点后才发布 grid_map_vis"
echo "   -> 解决: 在 rviz 中使用 Goal3DTool 点击目标位置"
echo ""
echo "2. 如果 /random_complex/global_map 没有数据"
echo "   -> 检查 random_complex 节点日志"
echo "   -> 尝试重启: rosnode kill /random_complex"
echo ""
echo "3. 临时解决方案: 手动添加 PointCloud2 显示"
echo "   -> 在 rviz 中 Add -> PointCloud2"
echo "   -> Topic 设置为: /random_complex/global_map"
echo "=========================================="
