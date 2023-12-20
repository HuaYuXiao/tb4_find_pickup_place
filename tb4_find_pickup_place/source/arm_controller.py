import rclpy
from rclpy.node import Node
from interbotix_xs_msgs.msg import JointSingleCommand, JointGroupCommand
from sensor_msgs.msg import JointState
import numpy as np
import time
import modern_robotics as mr
from interbotix_xs_modules.xs_robot import mr_descriptions as mrd
import math
import array
from scipy.spatial.transform import Rotation as R

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PoseArray
from rclpy.duration import Duration
from pan_tilt_msgs.msg import PanJointState,PanTiltCmdDeg


class ArmController(Node):
    def __init__(self):
        super().__init__("ArmController")
        self.cmd_pub = self.create_publisher(JointSingleCommand, "/px100/commands/joint_single", 10)
        self.group_pub = self.create_publisher(JointGroupCommand, "/px100/commands/joint_group", 10)
        self.pantil_deg_cmd = PanTiltCmdDeg()
        self.pantil_pub = self.create_publisher(PanTiltCmdDeg,"/pan_tilt_cmd_deg",10)
        self.fb_sub = self.create_subscription(JointState, "/joint_states", self.js_cb, 10)
        self.pub_timer = self.create_timer(0.1, self.timers_cb)
        #self.marker_sub = self.create_subscription(PoseArray,"/aruco_poses",self.ar_cb,10)
        self.ar_pos = None
        self.ar_quat = None
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

    def js_cb(self, msg):
        if len(msg.name) == 7:
            self.joint_pos.clear()
            for i in range(7):
                self.joint_pos.append(msg.position[i])

    # demo
    def timers_cb(self):
        if len(self.joint_pos) == 7:
            print(self.machine_state)
            match self.machine_state:
                case "INIT":
                    self.pantil_deg_cmd.pitch = 10.0
                    self.pantil_deg_cmd.yaw = 0.0
                    self.pantil_deg_cmd.speed = 10
                    self.pantil_pub.publish(self.pantil_deg_cmd)
                    if self.go_home_pos() == True and self.release():
                        print('go home pos done!')
                        #T_fk = self.joint_to_pose([0.0, 0.0, 0.0, 0.0])  # forward k 
                        #print('fk', T_fk)

                        # [[1.       0.       0.       0.248575]
                        #  [0.       1.       0.       0.      ]
                        #  [0.       0.       1.       0.1931  ]
                        #  [0.       0.       0.       1.      ]]

                        #self.machine_state = "NEXT1"
                        self.machine_state = "NEXT1"
                        time.sleep(3.0)
                case "NEXT1":
                    if self.go_sleep_pos() == True :
                        print('go sleep pos done!')
                        T_fk = self.joint_to_pose([-1.4, -0.35, 0.7, 1.0])  # forward k 
                        print('fk_1111111', T_fk)
                        #    [[ 0.03722394  0.98544973  0.16584091  0.01995401]
                        #     [-0.21582008  0.16996714 -0.96152632 -0.11569103]
                        #     [-0.97572336  0.          0.21900669  0.05393113]
                        #     [ 0.          0.          0.          1.        ]]
                        self.machine_state = "NEXT3"
                        time.sleep(3.0)
                case "NEXT2":
                    # if self.set_group_pos([1.0, 0.0, 0.5, -0.6]) == True and self.grasp(-0.5):
                    if self.set_group_pos([0.0, 0.0, -0.7, 0.0]) == True and self.grasp(0.7):
                        print('NEXT2 control done!')
                        self.machine_state = "NEXT4"
                        time.sleep(1.0)
                    pass
                case "NEXT3":
                    print('hahahahahahah')
                    # T_sd = np.array([
                    #                 [0.03, 0.99, 0.17, 0.02],
                    #                 [-0.22, 0.17, -0.96, -0.1],
                    #                 [-0.98,  0.0, 0.22, 0.05],
                    #                 [ 0.0, 0.0, 0.0, 1.0]])

                    # T_sd = np.array([[1. ,      0.  ,     0.  ,     0.248575],
                    #      [0.  ,     1.   ,    0. ,      0.      ],
                    #      [0.  ,     0.   ,    1.  ,     0.1931  ],
                    #      [0.   ,    0.   ,    0.  ,     1.      ]])

                                     
                    # T_sd = np.array([[ 0.0073467,  -0.96587625, -0.25889978, -0.01516348],
                    #                     [-0.98651051,  0.03534029, -0.15983768,  0.04872446],
                    #                     [ 0.16353301,  0.25658163, -0.95258743,  0.49120052],
                    #                     [ 0.0,          0.0,          0.0,          1.0        ]])

                    # T_sd = np.array([[ 0.02422121,  0.03237774, -0.99918217,  0.23682376],
                    #                  [ 0.01541115,  0.99934454,  0.03275658, -0.02277646],
                    #                  [ 0.99958783, -0.01619195,  0.02370636,  0.14766343],
                    #                  [ 0.        ,  0.        ,  0.        ,  1.        ]])

                    # T_sd = np.array([[ 1.        ,  0.0       ,  0.0       ,  0.23682376],
                    #                  [ 0.0       ,  1.0       ,  0.0       , -0.00000022],
                    #                  [ 0.0       ,  0.0       ,  1.0       ,  0.14766343],
                    #                  [ 0.        ,  0.        ,  0.        ,  1.        ]])
                    
                    # T_sd = np.array([[9.96101711e-01,  2.24534092e-02 ,-8.53066386e-02,  2.19109095e-01],
                    #                 [ 2.42875705e-02,  9.99494311e-01 ,-2.05240265e-02, -3.45613683e-04],
                    #                 [ 8.48026656e-02 ,-2.25159089e-02, 9.96143332e-01 , 1.47951060e-01],
                    #                 [ 0.00000000e+00 , 0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
                    
                    # T_sd = np.array([[ 0.85268753, -0.52241801,  0.0018422,   0.18950104],
                    #                  [ 0.52240162,  0.85261959, -0.01167866,  0.06729021],
                    #                  [ 0.00453045,  0.01092061,  0.9999301,   0.17875418],
                    #                  [ 0.,          0.,          0.   ,       1.        ]])
                    T_sd = np.array([[ 0.99911196, -0.03122706, -0.028287  ,  0.26839074],
                                     [ 0.03180284,  0.99929125,  0.020139  , -0.03359414],
                                     [ 0.02763807, -0.02102072,  0.99939695,  0.28844533],
                                     [ 0.        ,  0.        ,  0.        ,  1.        ]])

                    mlist, mflag = self.matrix_control(T_sd) # inverse
                    self.fk = self.joint_to_pose(mlist) # 由关节量算正运动学
                    print('fk', self.fk) # self.fk应该和T_sd差的不多
                    print('mlist:', mlist)
                    if mflag == True and self.release():
                        print('matrix control done!')
                        self.machine_state = "NEXT4"
                        time.sleep(3.0)
        pass

    def quat2matrix(self,quat,pos):
        position = pos[:,np.newaxis]
        share_vector = np.array([0,0,0,1],dtype = float)[np.newaxis,:]
        r = R.from_quat(quat)
        rotation_matrix = r.as_matrix()
        m34 = np.concatenate((rotation_matrix, position),axis = 1)
        matrix = np.concatenate((m34,share_vector),axis = 0)
        return matrix

    def cam2arm(self):
        '''
        need to write
        :return:  transform matrix between camera and arm
        '''
        return None

    def cam_pos_to_arm_pos(self):
        '''
        need to write
        :return:  position from camera coordinate to arm coordinate
        '''
        return None

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
                    print('current group pos:', check_pos)
                    if np.abs(pos_list[0] - check_pos[0]) < thred and np.abs(pos_list[1] - check_pos[1]) < thred and np.abs(pos_list[2] - check_pos[2]) < thred and np.abs(pos_list[3] - check_pos[3]) < thred:
                        return True
                    else:
                        if np.abs(pos_list[0] - check_pos[0]) >= thred:
                            print('waist moving...')
                        if np.abs(pos_list[1] - check_pos[1]) >= thred:
                            print('shoulder moving...')
                        if np.abs(pos_list[2] - check_pos[2]) >= thred:
                            print('elbow moving...')
                        if np.abs(pos_list[3] - check_pos[3]) >= thred:
                            print('wrist moving...')
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
                #eomg=0.001,
                eomg=0.2,
                ev=0.2,
            )
            solution_found = True
            print('success',success, solution_found)
            # Check to make sure a solution was found and that no joint limits were violated
            if success:
                print('success',success)
                theta_list = self._wrap_theta_list(theta_list)
                # solution_found = self._check_joint_limits(theta_list)
                solution_found = True
            else:
                solution_found = False
                print('no solution1111')

            if solution_found:
                if execute:
                    joint_list = [theta_list[0],theta_list[1],theta_list[2], theta_list[3]]
                    self.set_group_pos(joint_list) # kong zhi yun dong de 
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
        state = self.set_single_pos('waist', pos)
        return state
    
    def shoulder_control(self, pos):
        """
        lower limit = -0.4
        upper limit = ~0.9
        """
        pos = float(pos)
        state = self.set_single_pos('shoulder', pos)
        return state
    
    def elbow_control(self, pos):
        '''
        lower limit = -1.1
        upper limit = 0.8
        '''
        pos = float(pos)
        state = self.set_single_pos('elbow', pos)
        return state
    
    def wrist_control(self, pos):
        '''
        lower limit = -1.4
        upper limit = 1.8
        '''
        pos = float(pos)
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



def main():
    rclpy.init(args=None)
    contoller = ArmController()
    rclpy.spin(contoller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
