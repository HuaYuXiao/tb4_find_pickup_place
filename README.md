# turtlebot4: find, pickup and place



## 基本指令

### 启动接口

```bash
tony@iqr-turtlebot4-121:~$ ros2 launch iqr_tb4_bringup bringup.launch.py
```

### 加载地图

```bash
tony@iqr-turtlebot4-121:~$ rviz2
```

```bash
tony@iqr-turtlebot4-121:~$ ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=map.yaml
``

```bash
tony@iqr-turtlebot4-121:~$ ros2 run nav2_util lifecycle_bringup map_server
```

![image](https://github.com/HuaYuXiao/turtlebot2_pickup_and_place/assets/117464811/16008ebc-f038-4634-a5e8-1883e577c0b6)





