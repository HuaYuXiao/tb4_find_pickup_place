import rclpy
from rclpy.node import Node
from interbotix_xs_msgs.msg import JointSingleCommand, JointGroupCommand
from sensor_msgs.msg import JointState
import numpy as np
import time
import modern_robotics as mr
from interbotix_xs_modules.xs_robot import mr_descriptions as mrd
from scipy.spatial.transform import Rotation as R
import numpy as np
import math
import array
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import PoseArray
from pan_tilt_msgs.msg import PanTiltCmdDeg
from tf2_ros.transform_listener import TransformListener


class ArmController(Node):
    def __init__(self):
        super().__init__("ArmController")

        self.marker2camera_Matrix = np.eye(4)
        # self.camera2base_Matrix = np.array([[0.98480776, 0.0, -0.17364815, -0.0788152829],
        #                                     [0.0, 1.0, 0.0, -0.0175],
        #                                     [0.17364815, 0.0, 0.98480776, -0.268805398],
        #                                     [0.0, 0.0, 0.0, 1.0]])
        self.camera2base_Matrix = np.array([[ 9.99999924e-01,  3.49065785e-04, -1.74532885e-04, -3.88525342e-02],
                                            [-3.49065780e-04,  9.99999939e-01,  6.09234621e-08,  1.74842416e-02],
                                            [ 1.74532895e-04,  6.61744490e-24,  9.99999985e-01,  2.87007879e-01],
                                            [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

# [[ 9.84807697e-01  3.43762693e-04 -1.73648153e-01  7.88152829e-02]
#  [-3.49065785e-04  9.99999939e-01 -3.38813179e-21 -1.75293215e-02]
#  [ 1.73648142e-01  6.06146288e-05  9.84807757e-01 -2.68805398e-01]
#  [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]

        self.marker2base_Matrix = np.eye(4)
        self.rot_matrix1 = np.array([[0, 0, 1, 0],
                                     [0, 1, 0, 0],
                                     [-1, 0, 0, 0],
                                     [0, 0, 0, 1]])
        self.rot_matrix2 = np.array([[0, 0, 1, 0],
                                     [-1, 0, 0, 0],
                                     [0, -1, 0, 0],
                                     [0, 0, 0, 1]])
    
        self.fb_sub = self.create_subscription(JointState, "/joint_states", self.js_cb, 10)
        self.marker_sub = self.create_subscription(PoseArray,"/aruco_poses",self.ar_cb, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # self.read_timer = self.create_timer(0.01,self.timer_cb)

        self.cmd_pub = self.create_publisher(JointSingleCommand, "/px100/commands/joint_single", 10)
        self.group_pub = self.create_publisher(JointGroupCommand, "/px100/commands/joint_group", 10)
        self.pub_timer = self.create_timer(0.1, self.timers_cb)
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
        # print('joint satte callback')
        if len(msg.name) == 7:
            self.joint_pos.clear()
            for i in range(7):
                self.joint_pos.append(msg.position[i])


    # camera2marker matrix
    def ar_cb(self, msg):
        print('aruco callback')
        # 这里处理接收到的坐标信息
        pose = msg.poses[0]

        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        # 从 Pose 消息中获取方向信息
        orientation = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        matrix = self.quat2matrix(orientation, position)

        self.marker2camera_Matrix = matrix
        print("camera2marker matrix: ", self.marker2camera_Matrix)

        self.marker2base_Matrix = np.dot(self.camera2base_Matrix, np.dot(self.rot_matrix2, np.dot(self.marker2camera_Matrix, self.rot_matrix1)))
        # print('base2marker matrix:', self.base2marker_Matrix)

        return None


    # base2camera matrix
    # def timer_cb(self):
    #     try:
    #         now = rclpy.time.Time()
    #         # 从base到camera
    #         trans = self.tf_buffer.lookup_transform("camera_link", "px100/base_link", now)

    #         position = trans.transform.translation
    #         position = np.array([position.x, position.y, position.z])

    #         orientation = trans.transform.rotation
    #         orientation = np.array([orientation.x, orientation.y, orientation.z, orientation.w])

    #         matrix = self.quat2matrix(orientation, position)

    #         self.camera2base_Matrix = matrix
    #         # print("base2camera matrix: ", matrix)

    #     except :
    #         pass

    #     pass


    def timers_cb(self):
        # print('timers callback')
        if len(self.joint_pos) == 7:
            print(self.machine_state)
            match self.machine_state:
                case "INIT":
                    self.pantil_deg_cmd.pitch = 10.0
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
                            self.machine_state = "NEXT1"
                            # time.sleep(3.0)

                case "NEXT1":
                    print('base2marker_Matrix: ', self.marker2base_Matrix)

                    mlist, mflag = self.matrix_control(self.marker2base_Matrix)
                    self.fk = self.joint_to_pose(mlist)
                    # print('fk', self.fk)
                    # print('mlist:', mlist)
                    if mflag == True and self.release():
                        # print('matrix control done!')
                        self.machine_state = "NEXT2"
                        # time.sleep(3.0)
                case "NEXT2":
                    if self.set_group_pos([0.0, 0.0, -0.7, 0.0]) == True and self.grasp(0.0):
                        print('NEXT2 control done!')
                        self.machine_state = "NEXT3"
                        time.sleep(1.0)
                case "NEXT3":
                    if self.go_sleep_pos() == True and self.release():
                        print('NEXT3 control done!')
                        self.machine_state = "NEXT4"
                        time.sleep(1.0)
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
                eomg=0.001,
                ev=0.01,
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



def main():
    rclpy.init(args=None)
    contoller = ArmController()
    rclpy.spin(contoller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
