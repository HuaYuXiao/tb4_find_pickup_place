# TurtleBot4: Find, Pickup & Place

南方科技大学课程《EE211机器人感知与智能》期末项目

## 基本指令

### 启动接口

```bash
tony@iqr-turtlebot4-121:~$ ros2 launch iqr_tb4_bringup bringup.launch.py
```

### 加载地图

Open a terminal and launch `rviz2`:

```bash
tony@iqr-turtlebot4-121:~$ rviz2
```

Then, in another terminal, launch `map_server`:

```bash
tony@iqr-turtlebot4-121:~$ ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=map.yaml
```

Then, in another terminal, launch :

```bash
tony@iqr-turtlebot4-121:~$ ros2 run nav2_util lifecycle_bringup map_server
```

![image](https://github.com/HuaYuXiao/turtlebot2_pickup_and_place/assets/117464811/16008ebc-f038-4634-a5e8-1883e577c0b6)


## 参考教程

- https://turtlebot.github.io/turtlebot4-user-manual/tutorials/navigation.html
- https://fishros.org.cn/forum/topic/303/ros2-%E5%9F%BA%E7%A1%80-navigation2%E5%AF%BC%E8%88%AA%E7%B3%BB%E7%BB%9F
- https://blog.csdn.net/m0_65304012/article/details/128303733
- https://navigation.ros.org/tutorials/docs/navigation2_on_real_turtlebot3.html

## 团队贡献


