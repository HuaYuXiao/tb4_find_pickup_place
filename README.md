# turtlebot2_pickup_and_place



## 基本指令

### 加载地图

```bash
tony@iqr-turtlebot4-121:~$ rviz2
[INFO] [1702955906.833315712] [rviz2]: Stereo is NOT SUPPORTED
[INFO] [1702955906.833401372] [rviz2]: OpenGl version: 4.5 (GLSL 4.5)
[INFO] [1702955906.845324753] [rviz2]: Stereo is NOT SUPPORTED
```

```bash
tony@iqr-turtlebot4-121:~$ ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=map.yaml
[INFO] [1702955821.800115199] [map_server]: 
	map_server lifecycle node launched. 
	Waiting on external lifecycle transitions to activate
	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[INFO] [1702955821.800209796] [map_server]: Creating
[INFO] [1702955828.462949522] [map_server]: Configuring
[INFO] [map_io]: Loading yaml file: map.yaml
[DEBUG] [map_io]: resolution: 0.05
[DEBUG] [map_io]: origin[0]: -3.8
[DEBUG] [map_io]: origin[1]: -14.5
[DEBUG] [map_io]: origin[2]: 0
[DEBUG] [map_io]: free_thresh: 0.25
[DEBUG] [map_io]: occupied_thresh: 0.65
[DEBUG] [map_io]: mode: trinary
[DEBUG] [map_io]: negate: 0
[INFO] [map_io]: Loading image_file: ./map.pgm
[DEBUG] [map_io]: Read map ./map.pgm: 220 X 318 map @ 0.05 m/cell
[INFO] [1702955828.475576579] [map_server]: Activating
[INFO] [1702955828.475745398] [map_server]: Creating bond (map_server) to lifecycle manager.
```

```bash
tony@iqr-turtlebot4-121:~$ ros2 run nav2_util lifecycle_bringup map_server
[INFO] [1702956104.840330822] [map_server_lifecycle_client_30054474]: Waiting for service map_server/get_state...
```




