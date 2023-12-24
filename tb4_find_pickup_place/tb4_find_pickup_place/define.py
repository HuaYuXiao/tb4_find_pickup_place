from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
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
import math
import array
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import PoseArray
from pan_tilt_msgs.msg import PanTiltCmdDeg
from tf2_ros.transform_listener import TransformListener


PointS = PoseStamped(
    header=Header(frame_id='map'),
    pose=Pose(
        position=Point(x=0.25, y=-0.3, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    )
)

PointA = PoseStamped(
    header=Header(frame_id='map'),
    pose=Pose(
        position=Point(x=0.2, y=-3.2, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    )
)

PointB = PoseStamped(
    header=Header(frame_id='map'),
    pose=Pose(
        position=Point(x=2.85, y=-3.25, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    )
)


def nav_thread(navigator, goal_pose):
    navigator.waitUntilNav2Active()
    # rclpy.spin(navigator)
    navigator.goToPose(goal_pose)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback:
            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=30.0):
                navigator.cancelTask()

    # Do something depending on the return code
    resultS = navigator.getResult()
    if resultS == TaskResult.SUCCEEDED:
        print('Goal reached')
    elif resultS == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif resultS == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')


class ArmController(Node):
    def __init__(self, action=True):
        super().__init__("ArmController")
        '''
        action用于判断机械臂行为
        True：抓取
        False：放下
        '''
        self.action = action

        self.marker2camera_Matrix = np.eye(4)
        self.camera2base_Matrix = np.eye(4)
        self.marker2base_Matrix = np.eye(4)
        self.rot_matrix1 = np.array([[1, 0, 0, -0.01],
                                     [0, 1, 0, 0],
                                     [0, 0, 1, 0],
                                     [0, 0, 0, 1]])
        self.rot_matrix2 = np.array([[0, 0, 1, 0],
                                     [-1, 0, 0, 0],
                                     [0, -1, 0, 0],
                                     [0, 0, 0, 1]])
    
        self.fb_sub = self.create_subscription(JointState, "/joint_states", self.js_cb, 10)
        self.marker_sub = self.create_subscription(PoseArray,"/aruco_poses",self.ar_cb, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(JointSingleCommand, "/px100/commands/joint_single", 10)
        self.group_pub = self.create_publisher(JointGroupCommand, "/px100/commands/joint_group", 10)
        
        # 根据action采取不同的操作
        if self.action:
            self.pub_timer = self.create_timer(0.1, self.pickup_cb)
        else:
            self.pub_timer = self.create_timer(0.1, self.place_cb)

        self.pantil_pub = self.create_publisher(PanTiltCmdDeg,"/pan_tilt_cmd_deg",10)

        self.ar_pos = None
        self.ar_quat = None
        self.pantil_deg_cmd = PanTiltCmdDeg()
        self.arm_command = JointSingleCommand()
        self.arm_group_command = JointGroupCommand()
        
        self.cnt = 0
        self.thred = 0.1
        self.joint_pos = []
        self.moving_time = 2.0
        self.num_joints = 4
        self.joint_lower_limits = [-1.5, -0.4, -1.1, -1.4]
        self.joint_upper_limits = [1.5, 0.9, 0.8, 1.8]
        self.initial_guesses = [[0.0] * self.num_joints] * 3
        self.initial_guesses[1][0] = np.deg2rad(-30)
        self.initial_guesses[2][0] = np.deg2rad(30)
        self.robot_des: mrd.ModernRoboticsDescription = getattr(mrd, 'px100')

        self.machine_state = "INIT"

        self.gripper_pressure: float = 0.5
        self.gripper_pressure_lower_limit: int = 150
        self.gripper_pressure_upper_limit: int = 350
        self.gripper_value = self.gripper_pressure_lower_limit + (self.gripper_pressure*(self.gripper_pressure_upper_limit - self.gripper_pressure_lower_limit))

        pass


    def quat2matrix(self,quat,pos):
        position = pos[:,np.newaxis]
        share_vector = np.array([0,0,0,1],dtype = float)[np.newaxis,:]
        r = R.from_quat(quat)
        rotation_matrix = r.as_matrix()
        m34 = np.concatenate((rotation_matrix, position),axis = 1)
        matrix = np.concatenate((m34,share_vector),axis = 0)

        return matrix


    def js_cb(self, msg):
        # if self.pickupFlag == False or self.placeFlag == False:
        #     return

        # print('joint satte callback')
        if len(msg.name) == 7:
            self.joint_pos.clear()
            for i in range(7):
                self.joint_pos.append(msg.position[i])


    # camera2marker matrix
    def ar_cb(self, msg):
        # 更新相机与base的转换
        now = rclpy.time.Time()
        try:
            trans = self.tf_buffer.lookup_transform("px100/base_link","camera_link",now)
            position = trans.transform.translation
            orientation = trans.transform.rotation
            position = np.array([position.x, position.y, position.z])
            orientation = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
            self.camera2base_Matrix = self.quat2matrix(orientation, position)
            # print("camera2base updated")
        except :
            print("pass")
            pass

        # 这里处理接收到的坐标信息
        pose = msg.poses[0]

        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        # 从 Pose 消息中获取方向信息
        orientation = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        matrix = self.quat2matrix(orientation, position)

        self.marker2camera_Matrix = matrix
        # print("camera2marker matrix: ", self.marker2camera_Matrix)

        self.marker2base_Matrix = np.dot(self.camera2base_Matrix, np.dot(self.rot_matrix2, np.dot(self.marker2camera_Matrix, self.rot_matrix1)))
        # print('base2marker matrix:', self.base2marker_Matrix)
        #print('m1 ', self.marker2camera_Matrix)
        #print('m2 ', self.camera2base_Matrix)
        return None


    def pickup_cb(self):
        # print('timers callback')
        if len(self.joint_pos) == 7:
            # print(self.machine_state)
            match self.machine_state:
                case "INIT":
                    self.pantil_deg_cmd.pitch = 10.0  # 调整云台俯仰角
                    self.pantil_deg_cmd.yaw = 0.0
                    self.pantil_deg_cmd.speed = 10
                    self.pantil_pub.publish(self.pantil_deg_cmd)
                    if self.go_sleep_pos() == True and self.release():
                        print('go slep pos done!')

                        # print('base2marker_Matrix: ', self.base2marker_Matrix)
                        if np.array_equal(self.marker2base_Matrix, np.eye(4)):
                            print('no marker detected!')
                            self.machine_state = "INIT"
                            # time.sleep(3.0)
                        else:
                            print('marker detected!')
                            print('base2marker_Matrix: ', self.marker2base_Matrix)
                            self.machine_state = "NEXT0"
                            #self.machine_state = "NEXT4"
                            # time.sleep(3.0)

                case "NEXT0":
                    if self.set_group_pos([-1.0, 0.0, -1.0, 0.0]) == True:
                        print('NEXT0 control done!')
                        self.machine_state = "NEXT1"
                
                # case "NEXT1":
                #     if self.set_group_pos([0.0, 0.0, -1.0, 0.0]) == True:
                #         print('NEXT1 control done!')
                #         self.machine_state = "NEXT2"
                        
                case "NEXT1":
                    height_Matrix = np.array([[1, 0, 0, 0],
                                     [0, 1, 0, 0],
                                     [0, 0, 1, 0.1],
                                     [0, 0, 0, 1]])
                    mlist, mflag = self.matrix_control(np.dot(self.marker2base_Matrix, height_Matrix))
                    self.fk = self.joint_to_pose(mlist)
                    np.set_printoptions(precision=3)
                    # print('fk', self.fk)
                    print('mlist:', mlist)
                    error = self.marker2base_Matrix - self.fk
                    # error = np.random.random(4)
                    np.set_printoptions(precision=3)
                    print('error: ', error)
                    # if mflag == True and self.release():
                    if self.set_group_pos([mlist[0], mlist[1], mlist[2], mlist[3]]) and self.release():
                        print('NEXT1 done!')
                        # time.sleep(3.0)
                        self.grasp(0.7)
                        self.machine_state = "NEXT2"
                        time.sleep(1.0)

                case "NEXT2":
                    print('base2marker_Matrix: ', self.marker2base_Matrix)

                    mlist, mflag = self.matrix_control(self.marker2base_Matrix)
                    self.fk = self.joint_to_pose(mlist)
                    np.set_printoptions(precision=3)
                    # print('fk', self.fk)
                    print('mlist:', mlist)
                    error = self.marker2base_Matrix - self.fk
                    # error = np.random.random(4)
                    np.set_printoptions(precision=3)
                    print('error: ', error)
                    # if mflag == True and self.release():
                    if self.set_group_pos([mlist[0], mlist[1], mlist[2], mlist[3]]) and self.release():
                        print('matrix control done!')
                        # time.sleep(3.0)
                        self.grasp(0.7)
                        self.machine_state = "NEXT3"
                        time.sleep(1.0)
                
                case "NEXT3":
                    if self.set_group_pos([0.0, 0.0, -0.7, 0.0]) == True:
                        print('NEXT3 control done!')
                        self.machine_state = "NEXT4"
                        # time.sleep(3.0)
                case "NEXT4":
                    if self.go_sleep_pos() == True:
                        print('NEXT4 control done!')
                        # self.release()
                        self.machine_state = "NEXT6"
                        time.sleep(1.0)

                # case "NEXT5": # 循环模式，一直识别，跟随marker，有点卡
                #     print('base2marker_Matrix: ', self.marker2base_Matrix)

                #     mlist, mflag = self.matrix_control(self.marker2base_Matrix)
                #     self.fk = self.joint_to_pose(mlist)
                #     # print('fk', self.fk)
                #     # print('mlist:', mlist)
                #     if mflag == True and self.release():
                #         # print('matrix control done!')
                #         self.machine_state = "NEXT5"
                #         # time.sleep(3.0)
                        
                case "NEXT6":
                    if self.go_home_pos() and self.release() == True:
                        print('NEXT6 control done!')
                        self.machine_state = "over"
                        time.sleep(3.0)

        pass


    def place_cb(self):  # 卡不可能在这里面卡
        if len(self.joint_pos) == 7:
            match self.machine_state:
                case "INIT":
                    if self.go_home_pos() == True:
                        print('Go home pos done!')
                        self.machine_state = "NEXT0"
                case "NEXT0":
                    if self.release():
                        print('Release done!')  
                        self.machine_state = "NEXT1"                      
                case "NEXT1":
                    if self.go_sleep_pos() == True:
                        print('Go sleep pos done!')
                        print('place is done')
        pass    


    def set_single_pos(self, name, pos, blocking=True):
        '''
        ### @param: name: joint name
        ### @param: pos: radian
        ### @param: blocking - whether the arm need to check current position 

        '''
        self.arm_command.name = name
        self.arm_command.cmd = pos
        self.cmd_pub.publish(self.arm_command)

        thred = self.thred
        if blocking:
            check_pos = None
            cal_name = None
            if len(self.joint_pos) == 7:
                match name:
                    case "waist":
                        check_pos = self.joint_pos[0]
                        cal_name = 'joint'
                    case "shoulder":
                        check_pos = self.joint_pos[1]
                        cal_name = 'joint'
                    case "elbow":
                        check_pos = self.joint_pos[2]
                        cal_name = 'joint'
                    case "wrist_angle":
                        check_pos = self.joint_pos[3]
                        cal_name = 'joint'
                    case "gripper":
                        check_pos = self.joint_pos[4]
                        cal_name = 'gripper'
                    case _:
                        print('unvalid name input!')

                match cal_name:
                    case "joint":
                        dis = np.abs(pos-check_pos)
                        if dis < thred:
                            return True
                        else:
                            print('single joint moving...')
                            return False                       
                    case "gripper":
                        return True

        pass


    def set_group_pos(self, pos_list, blocking=True):
        '''
        ### @param: group pos: radian
        ### @param: blocking - whether the arm need to check current position 
        '''
        if len(pos_list) != self.num_joints:
            print('unexpect length of list!')
        else:
            self.arm_group_command.name = "arm"
            self.arm_group_command.cmd = pos_list
            self.group_pub.publish(self.arm_group_command)
     
            thred = self.thred
            if blocking:
                if len(self.joint_pos) == 7:
                    check_pos = self.joint_pos
                    # print('current group pos:', check_pos)
                    if np.abs(pos_list[0] - check_pos[0]) < thred and np.abs(pos_list[1] - check_pos[1]) < thred and np.abs(pos_list[2] - check_pos[2]) < thred and np.abs(pos_list[3] - check_pos[3]) < thred:
                        return True
                    else:
                        if np.abs(pos_list[0] - check_pos[0]) >= thred:
                            pass
                            # print('waist moving...')
                        if np.abs(pos_list[1] - check_pos[1]) >= thred:
                            pass
                            # print('shoulder moving...')
                        if np.abs(pos_list[2] - check_pos[2]) >= thred:
                            pass
                            # print('elbow moving...')
                        if np.abs(pos_list[3] - check_pos[3]) >= thred:
                            pass
                            # print('wrist moving...')
                            return False            
            pass


    def joint_to_pose(self, joint_state):
        return mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, joint_state)


    def go_home_pos(self):
        state = self.set_group_pos([0.0, 0.0, 0.0, 0.0])
        return state


    def go_sleep_pos(self):
        state = self.set_group_pos([-1.4, -0.35, 0.7, 1.0])
        return state


    def matrix_control(self, T_sd, custom_guess: list[float]=None, execute: bool=True):
        if custom_guess is None:
            initial_guesses = self.initial_guesses
        else:
            initial_guesses = [custom_guess]

        for guess in initial_guesses:
            theta_list, success = mr.IKinSpace(
                Slist=self.robot_des.Slist,
                M=self.robot_des.M,
                T=T_sd,
                thetalist0=guess,
                eomg=1.0,
                ev=0.005,
            )
            solution_found = True
            print('success: ',success)
            # Check to make sure a solution was found and that no joint limits were violated
            if success:
                print('success',success)
                theta_list = self._wrap_theta_list(theta_list)
                # solution_found = self._check_joint_limits(theta_list)
                solution_found = True
            else:
                solution_found = False

            if solution_found:
                if execute:
                    joint_list = [theta_list[0],theta_list[1],theta_list[2], theta_list[3]]
                    self.set_group_pos(joint_list)
                    self.T_sb = T_sd
                return theta_list, True

        # self.core.get_logger().warn('No valid pose could be found. Will not execute')
        return theta_list, False


    def waist_control(self, pos):
        """
        lower limit = -1.5
        upper limit = 1.5
        """
        pos = float(pos)

        if pos < -1.5:
            pos = -1.5
        elif pos > 1.5:
            pos = 1.5

        state = self.set_single_pos('waist', pos)
        return state


    def shoulder_control(self, pos):
        """
        lower limit = -0.4
        upper limit = ~0.9
        """
        pos = float(pos)

        if pos < -0.4:
            pos = -0.4
        elif pos > 0.9:
            pos = 0.9

        state = self.set_single_pos('shoulder', pos)
        return state
    
    def elbow_control(self, pos):
        '''
        lower limit = -1.1
        upper limit = 0.8
        '''
        pos = float(pos)

        if pos < -1.1:
            pos = -1.1
        elif pos > 0.8:
            pos = 0.8

        state = self.set_single_pos('elbow', pos)
        return state
    

    def wrist_control(self, pos):
        '''
        lower limit = -1.4
        upper limit = 1.8
        '''
        pos = float(pos)

        if pos < -1.4:
            pos = -1.4
        elif pos > 1.8:
            pos = 1.8

        state = self.set_single_pos('wrist_angle', pos)
        return state


    def gripper_controller(self, effort, delay: float):
        '''
        effort: release = 1.5
        effort: grasp = -0.6
        '''
        name = 'gripper'
        effort = float(effort)

        if len(self.joint_pos) == 7:
            gripper_state = self.set_single_pos(name, effort)
            time.sleep(delay)
            return gripper_state


    def set_pressure(self, pressure: float) -> None:
        """
        Set the amount of pressure that the gripper should use when grasping an object.
        :param pressure: a scaling factor from 0 to 1 where the pressure increases as
            the factor increases
        """
        self.gripper_value = self.gripper_pressure_lower_limit + pressure * (
            self.gripper_pressure_upper_limit - self.gripper_pressure_lower_limit
        )

    def release(self, delay: float = 1.0) -> None:
        """
        Open the gripper (when in 'pwm' control mode).
        :param delay: (optional) number of seconds to delay before returning control to the user
        """
        state = self.gripper_controller(1.5, delay)
        return state

    def grasp(self, pressure: float = 0.5, delay: float = 1.0) -> None:
        """
        Close the gripper (when in 'pwm' control mode).
        :param delay: (optional) number of seconds to delay before returning control to the user
        """
        state = self.gripper_controller(pressure, delay)
        return state


    def _wrap_theta_list(self, theta_list: list[np.ndarray]) -> list[np.ndarray]:
        """
        Wrap an array of joint commands to [-pi, pi) and between the joint limits.

        :param theta_list: array of floats to wrap
        :return: array of floats wrapped between [-pi, pi)
        """
        REV = 2 * np.pi
        theta_list = (theta_list + np.pi) % REV - np.pi
        for x in range(len(theta_list)):
            if round(theta_list[x], 3) < round(self.joint_lower_limits[x], 3):
                theta_list[x] += REV
            elif round(theta_list[x], 3) > round(self.joint_upper_limits[x], 3):
                theta_list[x] -= REV
        return theta_list


def arm_thread(controller, action):
    controller.action = action
    rclpy.spin(controller)
