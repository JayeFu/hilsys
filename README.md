# Launch Steps

启动gazebo仿真系统：
roslaunch iiwa_moveit whole_system.launch

运行move_group:
roslaunch iiwa_moveit move_group.launch

开始仿真:
roslaunch iiwa_moveit close_planning.launch

# Tips

下载之后需要先进行编译。以及检查iiwa_moveit/scripts目录下的py文件是否可执行