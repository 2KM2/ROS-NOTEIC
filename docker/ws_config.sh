# catkin 工作空间位置的唯一配置点,被 run.sh / build_ws.sh 共同 source。
#
# 仓库根挂载到容器的 /workspace。默认就把仓库根当作工作空间根:
#   /workspace          <- 工作空间根,build/ devel/ 生成在这里
#   /workspace/src      <- source space,catkin 会递归找到 src/hw_1/src/ 下的三个包
#
# 想改回"src/hw_1 作为工作空间根",执行时加 WS_SUBDIR=src/hw_1 即可。
: "${WS_SUBDIR:=}"

if [ -n "$WS_SUBDIR" ]; then
  WS="/workspace/$WS_SUBDIR"
else
  WS="/workspace"
fi
export WS WS_SUBDIR
