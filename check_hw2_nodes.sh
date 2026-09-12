#!/bin/bash
echo "=== 检查运行的节点 ==="
rosnode list

echo -e "\n=== 检查话题列表 ==="
rostopic list

echo -e "\n=== 检查地图话题 ==="
rostopic info /random_complex/global_map

echo -e "\n=== 检查地图是否有数据 ==="
timeout 2 rostopic echo /random_complex/global_map -n 1 2>&1 | head -20
