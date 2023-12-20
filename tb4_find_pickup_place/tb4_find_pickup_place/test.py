import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import numpy as np
import math
import array
from scipy.spatial.transform import Rotation as R

#订阅rviz2发出的相机识别到marker的消息，再转换成其次矩阵，相机到marker的矩阵

class ArucoPoseSubscriber(Node):
    def __init__(self):
        super().__init__('aruco_pose_subscriber')
        self.subscription = self.create_subscription(
            PoseArray,
            '/aruco_poses',  # 你要订阅的话题名称
            self.aruco_poses_callback,  # 回调函数，当有消息到达时调用
            10  # 消息队列大小
        )
        self.subscription  # 防止 Python 中 subscription 变量被垃圾回收

    def aruco_poses_callback(self, msg):
        # 这里处理接收到的坐标信息
        for pose in msg.poses:
            print(f"Aruco Pose: {pose}")

        position = np.array([pose.position.x, pose.position.y, pose.position.z])

            # 从 Pose 消息中获取方向信息
        orientation = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        matrix = self.quat2matrix(orientation, position)
        # 输出位置信息和方向信息矩阵
        print("Position Matrix:")
        print(position)

        print("Orientation Matrix:")
        print(orientation)

        print("Homo matrix")
        print(matrix)
        print()
        
        a = np.array([[ 0.98480776, -0.  ,       -0.17364815, 0.07881529],
                      [ 0.  ,        1.  ,       -0.        , -0.0175    ],
                      [ 0.17364815 , 0.  ,        0.98480776, -0.2688054 ],
                      [ 0.  ,        0.  ,        0.        ,  1.        ]])  # base to cam
    
        #matrix = np.linalg.inv(matrix)
        a = np.linalg.inv(a)
        
        rot_mat = np.array([[0, 0, 1, 0],
                            [-1, 0, 0, 0],
                            [0, -1, 0, 0],
                            [0, 0, 0, 1]])
        
        rot_mat2 = np.array([[0, 0, 1, 0],
                             [0, 1, 0, 0],
                             [-1, 0, 0, 0],
                             [0, 0, 0, 1]])
        
        
        homo2 = np.matmul(np.matmul(a, rot_mat), matrix)
        #print("homo:",homo2)
        homo2 = np.matmul(homo2,rot_mat2)
        #print("matrix: ", matrix)
        # print()
        print("homo: ",homo2)


    def quat2matrix(self,quat,pos):
        position = pos[:,np.newaxis]
        share_vector = np.array([0,0,0,1],dtype = float)[np.newaxis,:]
        r = R.from_quat(quat)
        rotation_matrix = r.as_matrix()
        m34 = np.concatenate((rotation_matrix, position),axis = 1)
        matrix = np.concatenate((m34,share_vector),axis = 0)
        return matrix
    

    


def main():
    rclpy.init()
    aruco_pose_subscriber = ArucoPoseSubscriber()
    rclpy.spin(aruco_pose_subscriber)
    aruco_pose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
