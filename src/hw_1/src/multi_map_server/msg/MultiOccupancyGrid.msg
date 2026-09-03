# 多张栅格地图 + 每张图各自的原点位姿。
# rviz_plugins/MultiProbMapDisplay 用的就是这个消息:
#   maps[k].info.{resolution,width,height,origin}、maps[k].data
#   origins[k].{position,orientation}
nav_msgs/OccupancyGrid[] maps
geometry_msgs/Pose[] origins
