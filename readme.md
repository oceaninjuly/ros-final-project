## 机器人导论：小车在未知环境的自主导航

​		**小组成员：刘佳瑜  黄浩洋   金思琪**

上述文件是在`ros noetic`下，`ros`工作目录中所创建的功能包，其中：

-  `exp`包含了所要测试的未知`gazebo`世界文件；

- `two_wheel_car`包含了机器人模型、机器人运行脚本等机器人运行数据；
- `my_global_planner`包含了自定义的全局路径规划算法，用于替换`move_base`中默认的算法。

将上述三个功能包放置到ros工作目录的`src`下，然后运行`catkin_make`编译工作目录即可。

另外，上述代码还用到了`ros`的`navigation`功能包，需要另外安装。

编译完成后，一键启动的命令为`roslaunch two_wheel_car gazebo_run.launch`，启动后，即可通过在弹出的rviz窗口中用`navi 2d`选点，来令机器人自主运动。

此外，也可以在启动后，运行`roslaunch two_wheel_car publisher.launch `来发布一个精确的导航点，其中具体的导航点信息在`two_wheel_car/param/points.yaml`中指定。