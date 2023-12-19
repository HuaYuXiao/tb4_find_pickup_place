# turtlebot2_pickup_and_place



## 基本指令

### 启动接口

```bash
tony@iqr-turtlebot4-121:~$ ros2 launch iqr_tb4_bringup bringup.launch.py
[INFO] [launch]: All log files can be found below /home/tony/.ros/log/2023-12-19-12-11-10-389755-iqr-turtlebot4-121-243261
[INFO] [launch]: Default logging verbosity is set to INFO
/opt/ros/humble/lib/python3.10/site-packages/launch_ros/events/lifecycle/lifecycle_node_matchers.py:30: UserWarning: 'matches_node_name' has been moved into the 'launch.events' module and will be removed from the 'lifecycle' module in the future
  warnings.warn(
/opt/ros/humble/lib/python3.10/site-packages/launch_ros/events/lifecycle/lifecycle_node_matchers.py:30: UserWarning: 'matches_node_name' has been moved into the 'launch.events' module and will be removed from the 'lifecycle' module in the future
  warnings.warn(
[INFO] [realsense2_camera_node-7]: process started with pid [243363]
[INFO] [xs_sdk-1]: process started with pid [243351]
[INFO] [robot_state_publisher-2]: process started with pid [243353]
[INFO] [serial_bridge-3]: process started with pid [243355]
[INFO] [modbus_rtu_driver-4]: process started with pid [243357]
[INFO] [rplidar_node-5]: process started with pid [243359]
[INFO] [PanTiltDriverNode-6]: process started with pid [243361]
[INFO] [static_transform_publisher-8]: process started with pid [243369]
[INFO] [static_transform_publisher-9]: process started with pid [243409]
[PanTiltDriverNode-6] pan-tilt ID:1 ,port name:/dev/pan_tilt
[xs_sdk-1] [INFO] Using Interbotix X-Series Driver Version: 'v0.3.3'.
[xs_sdk-1] [INFO] Using logging level 'INFO'.
[xs_sdk-1] [INFO] Loaded mode configs from '/home/tony/ros2_ws/install/interbotix_xsarm_control/share/interbotix_xsarm_control/config/modes.yaml'.
[xs_sdk-1] [INFO] Loaded motor configs from '/home/tony/ros2_ws/install/interbotix_xsarm_control/share/interbotix_xsarm_control/config/px100.yaml'.
[xs_sdk-1] [INFO] Pinging all motors specified in the motor_config file. (Attempt 1/3)
[rplidar_node-5] [INFO] [1702959071.297369298] [rplidar_node]: RPLidar running on ROS2 package rplidar_ros. RPLIDAR SDK Version:2.0.0
[xs_sdk-1] [INFO] 	Found DYNAMIXEL ID:  4, Model: 'XL430-W250', Joint Name: 'wrist_angle'.
[rplidar_node-5] [INFO] [1702959071.357289904] [rplidar_node]: RPLidar S/N: 71D8ED93C0EA98C7A0E69BF5E1064560
[rplidar_node-5] [INFO] [1702959071.357329199] [rplidar_node]: Firmware Ver: 1.32
[rplidar_node-5] [INFO] [1702959071.357333593] [rplidar_node]: Hardware Rev: 6
[xs_sdk-1] [INFO] 	Found DYNAMIXEL ID:  3, Model: 'XL430-W250', Joint Name: 'elbow'.
[rplidar_node-5] [INFO] [1702959071.408487996] [rplidar_node]: RPLidar health status : 0
[rplidar_node-5] [INFO] [1702959071.408533734] [rplidar_node]: RPLidar health status : OK.
[rplidar_node-5] [INFO] [1702959071.408538372] [rplidar_node]: Start
[xs_sdk-1] [INFO] 	Found DYNAMIXEL ID:  2, Model: 'XL430-W250', Joint Name: 'shoulder'.
[realsense2_camera_node-7] [INFO] [1702959071.461018121] [camera.camera]: RealSense ROS v4.54.1
[realsense2_camera_node-7] [INFO] [1702959071.461114913] [camera.camera]: Built with LibRealSense v2.54.1
[realsense2_camera_node-7] [INFO] [1702959071.461127061] [camera.camera]: Running with LibRealSense v2.54.1
[realsense2_camera_node-7] [INFO] [1702959071.472748711] [camera.camera]: Device with serial number 231522071827 was found.
[realsense2_camera_node-7] 
[realsense2_camera_node-7] [INFO] [1702959071.472809183] [camera.camera]: Device with physical ID /sys/devices/pci0000:00/0000:00:14.0/usb4/4-3/4-3:1.0/video4linux/video0 was found.
[realsense2_camera_node-7] [INFO] [1702959071.472818698] [camera.camera]: Device with name Intel RealSense D435I was found.
[realsense2_camera_node-7] [INFO] [1702959071.472964269] [camera.camera]: Device with port number 4-3 was found.
[realsense2_camera_node-7] [INFO] [1702959071.472975925] [camera.camera]: Device USB type: 3.2
[realsense2_camera_node-7] [INFO] [1702959071.473488605] [camera.camera]: getParameters...
[realsense2_camera_node-7] [INFO] [1702959071.473699404] [camera.camera]: JSON file is not provided
[realsense2_camera_node-7] [INFO] [1702959071.473711429] [camera.camera]: Device Name: Intel RealSense D435I
[realsense2_camera_node-7] [INFO] [1702959071.473719001] [camera.camera]: Device Serial No: 231522071827
[realsense2_camera_node-7] [INFO] [1702959071.473726053] [camera.camera]: Device physical port: /sys/devices/pci0000:00/0000:00:14.0/usb4/4-3/4-3:1.0/video4linux/video0
[realsense2_camera_node-7] [INFO] [1702959071.473733169] [camera.camera]: Device FW version: 5.15.0.2
[realsense2_camera_node-7] [INFO] [1702959071.473739810] [camera.camera]: Device Product ID: 0x0B3A
[realsense2_camera_node-7] [INFO] [1702959071.473746041] [camera.camera]: Sync Mode: Off
[xs_sdk-1] [INFO] 	Found DYNAMIXEL ID:  5, Model: 'XL430-W250', Joint Name: 'gripper'.
[realsense2_camera_node-7] [INFO] [1702959071.484415328] [camera.camera]: Set ROS param depth_module.profile to default: 848x480x30
[realsense2_camera_node-7] [INFO] [1702959071.489820413] [camera.camera]: Set ROS param rgb_camera.profile to default: 1280x720x30
[realsense2_camera_node-7] [INFO] [1702959071.490140306] [camera.camera]: Set ROS param gyro_fps to default: 200
[realsense2_camera_node-7] [INFO] [1702959071.490165814] [camera.camera]: Set ROS param accel_fps to default: 100
[realsense2_camera_node-7] [INFO] [1702959071.496542504] [camera.camera]: Stopping Sensor: Depth Module
[realsense2_camera_node-7] [INFO] [1702959071.507408383] [camera.camera]: Starting Sensor: Depth Module
[realsense2_camera_node-7] [INFO] [1702959071.516517344] [camera.camera]: Open profile: stream_type: Depth(0), Format: Z16, Width: 848, Height: 480, FPS: 30
[realsense2_camera_node-7] [INFO] [1702959071.516625974] [camera.camera]: Stopping Sensor: RGB Camera
[realsense2_camera_node-7] [INFO] [1702959071.517977833] [camera.camera]: Starting Sensor: RGB Camera
[xs_sdk-1] [INFO] 	Found DYNAMIXEL ID:  1, Model: 'XL430-W250', Joint Name: 'waist'.
[realsense2_camera_node-7] [INFO] [1702959071.530297955] [camera.camera]: Open profile: stream_type: Color(0), Format: RGB8, Width: 1280, Height: 720, FPS: 30
[realsense2_camera_node-7] [INFO] [1702959071.531582326] [camera.camera]: RealSense Node Is Up!
[realsense2_camera_node-7] [WARN] [1702959071.645810265] [camera.camera]: 
[serial_bridge-3] [INFO] [1702959071.650512378] [IoContext::IoContext]: Thread(s) Created: 2
[static_transform_publisher-9] [INFO] [1702959071.670315672] [right_wheel_drop_stf]: Spinning until stopped - publishing transform
[static_transform_publisher-9] translation: ('0.000000', '-0.116500', '0.040200')
[static_transform_publisher-9] rotation: ('-0.707073', '0.000000', '0.000000', '0.707141')
[static_transform_publisher-9] from 'base_link' to 'wheel_drop_right'
[robot_state_publisher-2] [WARN] [1702959071.670715168] [kdl_parser]: The root link base_link has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF.
[robot_state_publisher-2] [INFO] [1702959071.671065058] [robot_state_publisher]: got segment base_link
[robot_state_publisher-2] [INFO] [1702959071.671128209] [robot_state_publisher]: got segment bump_front_center
[robot_state_publisher-2] [INFO] [1702959071.671134100] [robot_state_publisher]: got segment bump_front_left
[robot_state_publisher-2] [INFO] [1702959071.671139341] [robot_state_publisher]: got segment bump_front_right
[robot_state_publisher-2] [INFO] [1702959071.671144606] [robot_state_publisher]: got segment bump_left
[robot_state_publisher-2] [INFO] [1702959071.671149666] [robot_state_publisher]: got segment bump_right
[robot_state_publisher-2] [INFO] [1702959071.671154647] [robot_state_publisher]: got segment bumper
[robot_state_publisher-2] [INFO] [1702959071.671159881] [robot_state_publisher]: got segment button_1
[robot_state_publisher-2] [INFO] [1702959071.671164864] [robot_state_publisher]: got segment button_2
[robot_state_publisher-2] [INFO] [1702959071.671169842] [robot_state_publisher]: got segment button_power
[robot_state_publisher-2] [INFO] [1702959071.671174969] [robot_state_publisher]: got segment camera_bottom_screw_frame
[robot_state_publisher-2] [INFO] [1702959071.671180473] [robot_state_publisher]: got segment camera_color_frame
[robot_state_publisher-2] [INFO] [1702959071.671185858] [robot_state_publisher]: got segment camera_color_optical_frame
[robot_state_publisher-2] [INFO] [1702959071.671191030] [robot_state_publisher]: got segment camera_depth_frame
[robot_state_publisher-2] [INFO] [1702959071.671196245] [robot_state_publisher]: got segment camera_depth_optical_frame
[robot_state_publisher-2] [INFO] [1702959071.671200981] [robot_state_publisher]: got segment camera_left_ir_frame
[robot_state_publisher-2] [INFO] [1702959071.671206546] [robot_state_publisher]: got segment camera_left_ir_optical_frame
[robot_state_publisher-2] [INFO] [1702959071.671215558] [robot_state_publisher]: got segment camera_link
[robot_state_publisher-2] [INFO] [1702959071.671224278] [robot_state_publisher]: got segment camera_right_ir_frame
[robot_state_publisher-2] [INFO] [1702959071.671229569] [robot_state_publisher]: got segment camera_right_ir_optical_frame
[robot_state_publisher-2] [INFO] [1702959071.671234843] [robot_state_publisher]: got segment cliff_front_left
[robot_state_publisher-2] [INFO] [1702959071.671240136] [robot_state_publisher]: got segment cliff_front_right
[robot_state_publisher-2] [INFO] [1702959071.671245726] [robot_state_publisher]: got segment cliff_side_left
[robot_state_publisher-2] [INFO] [1702959071.671250832] [robot_state_publisher]: got segment cliff_side_right
[robot_state_publisher-2] [INFO] [1702959071.671255987] [robot_state_publisher]: got segment front_caster_link
[robot_state_publisher-2] [INFO] [1702959071.671260931] [robot_state_publisher]: got segment imu_extra_link
[robot_state_publisher-2] [INFO] [1702959071.671266159] [robot_state_publisher]: got segment imu_link
[robot_state_publisher-2] [INFO] [1702959071.671275433] [robot_state_publisher]: got segment ir_intensity_front_center_left
[robot_state_publisher-2] [INFO] [1702959071.671284450] [robot_state_publisher]: got segment ir_intensity_front_center_right
[robot_state_publisher-2] [INFO] [1702959071.671289637] [robot_state_publisher]: got segment ir_intensity_front_left
[robot_state_publisher-2] [INFO] [1702959071.671294996] [robot_state_publisher]: got segment ir_intensity_front_right
[robot_state_publisher-2] [INFO] [1702959071.671300124] [robot_state_publisher]: got segment ir_intensity_left
[robot_state_publisher-2] [INFO] [1702959071.671305021] [robot_state_publisher]: got segment ir_intensity_right
[robot_state_publisher-2] [INFO] [1702959071.671309671] [robot_state_publisher]: got segment ir_intensity_side_left
[robot_state_publisher-2] [INFO] [1702959071.671314197] [robot_state_publisher]: got segment ir_omni
[robot_state_publisher-2] [INFO] [1702959071.671323163] [robot_state_publisher]: got segment laser_link
[robot_state_publisher-2] [INFO] [1702959071.671331838] [robot_state_publisher]: got segment left_wheel
[robot_state_publisher-2] [INFO] [1702959071.671337259] [robot_state_publisher]: got segment mouse
[robot_state_publisher-2] [INFO] [1702959071.671342697] [robot_state_publisher]: got segment pan_tilt_base_link
[robot_state_publisher-2] [INFO] [1702959071.671347977] [robot_state_publisher]: got segment pan_tilt_pitch_link
[robot_state_publisher-2] [INFO] [1702959071.671353205] [robot_state_publisher]: got segment pan_tilt_surface
[robot_state_publisher-2] [INFO] [1702959071.671358294] [robot_state_publisher]: got segment pan_tilt_yaw_link
[robot_state_publisher-2] [INFO] [1702959071.671363732] [robot_state_publisher]: got segment px100/base_link
[robot_state_publisher-2] [INFO] [1702959071.671369373] [robot_state_publisher]: got segment px100/ee_arm_link
[robot_state_publisher-2] [INFO] [1702959071.671374465] [robot_state_publisher]: got segment px100/ee_gripper_link
[robot_state_publisher-2] [INFO] [1702959071.671379371] [robot_state_publisher]: got segment px100/fingers_link
[robot_state_publisher-2] [INFO] [1702959071.671412455] [robot_state_publisher]: got segment px100/forearm_link
[robot_state_publisher-2] [INFO] [1702959071.671418851] [robot_state_publisher]: got segment px100/gripper_bar_link
[robot_state_publisher-2] [INFO] [1702959071.671424426] [robot_state_publisher]: got segment px100/gripper_link
[robot_state_publisher-2] [INFO] [1702959071.671429132] [robot_state_publisher]: got segment px100/gripper_prop_link
[robot_state_publisher-2] [INFO] [1702959071.671433725] [robot_state_publisher]: got segment px100/left_finger_link
[robot_state_publisher-2] [INFO] [1702959071.671438654] [robot_state_publisher]: got segment px100/right_finger_link
[robot_state_publisher-2] [INFO] [1702959071.671443831] [robot_state_publisher]: got segment px100/shoulder_link
[robot_state_publisher-2] [INFO] [1702959071.671448983] [robot_state_publisher]: got segment px100/upper_arm_link
[robot_state_publisher-2] [INFO] [1702959071.671458277] [robot_state_publisher]: got segment right_wheel
[robot_state_publisher-2] [INFO] [1702959071.671467669] [robot_state_publisher]: got segment top_fix_link
[robot_state_publisher-2] [INFO] [1702959071.671472725] [robot_state_publisher]: got segment wheel_drop_left
[robot_state_publisher-2] [INFO] [1702959071.671477897] [robot_state_publisher]: got segment wheel_drop_right
[static_transform_publisher-8] [INFO] [1702959071.680106759] [left_wheel_drop_stf]: Spinning until stopped - publishing transform
[static_transform_publisher-8] translation: ('0.000000', '0.116500', '0.040200')
[static_transform_publisher-8] rotation: ('-0.707073', '0.000000', '0.000000', '0.707141')
[static_transform_publisher-8] from 'base_link' to 'wheel_drop_left'
[modbus_rtu_driver-4] [INFO] [1702959071.709675630] [modbus_rtu_driver_node]: WitMotion ModbusRTU Driver
[xs_sdk-1] [WARN] Writing startup register values to EEPROM. This only needs to be done once on a robot if using a default motor config file, or after a motor config file has been modified. Can set `write_eeprom_on_startup` to false from now on.
[xs_sdk-1] [WARN] Could not get 'Goal_Current' Item Info. This message can be ignored if none of the robot's motors support current control.
[xs_sdk-1] [WARN] SyncWriteHandler for Goal_Current not added as it's not supported.
[rplidar_node-5] [INFO] [1702959072.097102036] [rplidar_node]: current scan mode: Sensitivity, sample rate: 16 Khz, max_distance: 16.0 m, scan frequency:10.0 Hz, 
[xs_sdk-1] [INFO] The operating mode for the 'arm' group was changed to 'position' with profile type 'time'.
[xs_sdk-1] [INFO] The operating mode for the 'gripper' joint was changed to 'position' with profile type 'velocity'.
[xs_sdk-1] [WARN] Could not get 'Goal_Current' Item Info. This message can be ignored if none of the robot's motors support current control.
[xs_sdk-1] [INFO] Interbotix X-Series Driver is up!
[xs_sdk-1] [INFO] [1702959072.402658959] [interbotix_xs_sdk.xs_sdk]: InterbotixRobotXS is up!
```

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




