# mecanum_robotsim

**About：ROS联合Webots开发之麦克纳姆轮篇**

**tutorial website：https://blog.csdn.net/xiaokai1999/article/details/112601720**

## 操作方法
1. 命令行进入catkin_ws/src
``` shell 
$ cd catkin_ws/src
$ git clone https://github.com/JackyMao1999/webots_demo.git
```
2. 编译
``` shell 
$ catkin_make
```
3. 移动机器人
``` shell
$ roslaunch mecanum_robotsim webots.launch 
$ rosrun mecanum_robotsim mecanum_velocity_v1
```
