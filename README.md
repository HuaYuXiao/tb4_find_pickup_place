# TurtleBot4: Navigation and Arm Control

南方科技大学课程《EE211机器人感知与智能》期末项目

![HitCount](https://img.shields.io/endpoint?url=https%3A%2F%2Fhits.dwyl.com%2FHuaYuXiao%2Ftb4-Navigation-and-Arm-Control.json%3Fcolor%3Dpink)
![Static Badge](https://img.shields.io/badge/ROS-humble-22314E?logo=ros)
![Static Badge](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu)
![Static Badge](https://img.shields.io/badge/Python-3.11.5-3776AB?logo=python)

在实验室环境中，从TurtleBot4无人车上搭载的RPLIDAR S3 SL-Dtof激光雷达对场地进行SLAM，之后控制无人车动态避障导航到指定区域，闭环控制环绕搜索并移动接近Aruco物块。最后采用自研的机械臂控制算法，调整到合适的姿态对物块进行抓取和摆放。

## Video on YouTube

https://youtu.be/JnZ-MSPzgaY

## 基本指令

### 屏幕显示

```bash
sudo systemctl stop gdm3
```

```bash
sudo /etc/NX/nxserver --restart
```

<img width="1440" alt="image" src="https://github.com/HuaYuXiao/tb4_find_pickup_place/assets/117464811/57fc1409-7d51-4a97-903b-279116c6501c">

`Ctrl`+`Alt`+`T`

![image](https://github.com/HuaYuXiao/tb4_find_pickup_place/assets/117464811/960671c7-d6c4-4607-a8cf-beb40eb28c71)

参考教程：

- https://zhuanlan.zhihu.com/p/519648451

### 启动接口

```bash
ros2 launch iqr_tb4_bringup bringup.launch.py
```

### 键盘控制

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 相机识别marker

注意：请确保环境照明条件良好，否则会严重影响marker识别率！

```bash
ros2 launch ros2_aruco aruco_recognition.launch.py
```

### 定位导航

Open a terminal and launch `localization`:

```bash
ros2 launch turtlebot4_navigation localization.launch.py map:=map.yaml
```

Then, in another terminal, launch `nav2`:

```bash
ros2 launch turtlebot4_navigation nav2.launch.py
```

In a new terminal launch `Rviz` so that you can view the map and interact with navigation:

```bash
ros2 launch turtlebot4_viz view_robot.launch.py
```

![image](https://github.com/HuaYuXiao/tb4_find_pickup_place/assets/117464811/ac469303-28e1-4fce-a1a9-4304e864e7ec)

Click on `2D Pose Estimate`, and then click and drag the arrow on the map to approximate the position and orientation of the robot.

![image](https://github.com/HuaYuXiao/tb4_find_pickup_place/assets/117464811/ee2ff89b-b51e-4383-aade-bf43532ec90f)

The `Nav2 Goal` tool allows you to set a goal pose for the robot. 

![image](https://github.com/HuaYuXiao/tb4_find_pickup_place/assets/117464811/34afbcba-49a8-44c0-b1a3-0485682772b7)

参考教程：

- https://turtlebot.github.io/turtlebot4-user-manual/tutorials/navigation.html
- https://github.com/turtlebot/turtlebot4_tutorials
- https://fishros.org.cn/forum/topic/303/ros2-%E5%9F%BA%E7%A1%80-navigation2%E5%AF%BC%E8%88%AA%E7%B3%BB%E7%BB%9F

### 加载地图

Open a terminal and launch `rviz2`:

```bash
rviz2
```

Then, in another terminal, launch `map_server`:

```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=map.yaml
```

Then, in another terminal, launch `lifecycle_bringup`:

```bash
ros2 run nav2_util lifecycle_bringup map_server
```

![image](https://github.com/HuaYuXiao/turtlebot2_pickup_and_place/assets/117464811/16008ebc-f038-4634-a5e8-1883e577c0b6)

参考教程：
- https://blog.csdn.net/m0_65304012/article/details/128303733

### 重启底盘

![image](https://github.com/HuaYuXiao/tb4_find_pickup_place/assets/117464811/94fc7bf5-0afc-49b5-aeec-12b42cc0d708)


## 团队贡献

- 岳翼遥：相机识别Marker
- 施永祺：机械臂逆运动学
- 华羽霄：全局导航，角度位置微调


## Citations

```bibtex
@InProceedings{macenski2020marathon2,
author = {Macenski, Steven and Martin, Francisco and White, Ruffin and Ginés Clavero, Jonatan},
title = {The Marathon 2: A Navigation System},
booktitle = {2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
year = {2020}
}
```
