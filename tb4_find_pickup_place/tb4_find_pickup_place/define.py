from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist
from std_msgs.msg import Header
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from threading import Thread, Event
import time
from rclpy.node import Node
from interbotix_xs_msgs.msg import JointSingleCommand, JointGroupCommand
from sensor_msgs.msg import JointState
import numpy as np
import modern_robotics as mr
from interbotix_xs_modules.xs_robot import mr_descriptions as mrd
from scipy.spatial.transform import Rotation as R
from math import radians
import datetime
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import PoseArray
from pan_tilt_msgs.msg import PanTiltCmdDeg
from tf2_ros.transform_listener import TransformListener
from tf_transformations import quaternion_from_euler


qA = quaternion_from_euler(0.0, 0.0, np.deg2rad(-90.0))
PointA = PoseStamped(
    header=Header(frame_id='map'),
    pose=Pose(
        position=Point(x=0.2, y=-3.3, z=0.0),
        orientation=Quaternion(x=qA[0], y=qA[1], z=qA[2], w=qA[3])
    )
)

qB = quaternion_from_euler(0.0, 0.0, np.deg2rad(0.0))
PointB = PoseStamped(
    header=Header(frame_id='map'),
    pose=Pose(
        position=Point(x=2.85, y=-3.2, z=0.0),
        orientation=Quaternion(x=qB[0], y=qB[1], z=qB[2], w=qB[3])
    )
)

qS = quaternion_from_euler(0.0, 0.0, np.deg2rad(-90.0))  
PointS = PoseStamped(
    header=Header(frame_id='map'),
    pose=Pose(
        position=Point(x=0.3, y=-0.3, z=0.0),
        orientation=Quaternion(x=qS[0], y=qS[1], z=qS[2], w=qS[3])
    )
)
