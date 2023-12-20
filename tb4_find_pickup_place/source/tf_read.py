from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import rclpy
from rclpy.node import Node
import numpy as np
import math
import array
from scipy.spatial.transform import Rotation as R



# 相机到base的变换
class TF_Reader(Node):
    def __init__(self):
        super().__init__("tf_reader")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.read_timer = self.create_timer(0.01,self.timer_cb)
        pass
    def timer_cb(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform("camera_link","px100/base_link",now)
            trans_2 = self.tf_buffer.lookup_transform("px100/base_link","camera_link",now)
            print("try")
            # trans = self.tf_buffer.lookup_transform("px100/base_link","",now)
            #print("trans1", trans)
            #print()
            position = trans_2.transform.translation
            orientation = trans_2.transform.rotation
            position = np.array([position.x, position.y, position.z])
            orientation = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
            matrix = self.quat2matrix(orientation, position)
            # print("Position Matrix:")
            # print(position)

            # print("Orientation Matrix:")
            # print(orientation)
            # print()
            print("Homo matrix")
            print(matrix)

            #print("trans2: ", trans_2)
            #print()
            position = trans.transform.translation
            orientation = trans.transform.rotation
            position = np.array([position.x, position.y, position.z])
            orientation = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
            matrix2 = self.quat2matrix(orientation, position)
            # print("Position Matrix:")
            # print(position)

            # print("Orientation Matrix:")
            # print(orientation)
            # print()
            # print("Homo matrix22")
            # print(matrix2)
            #print(matrix*matrix2)

        except :
            print("pass")
            pass

        pass
    def quat2matrix(self,quat,pos):
        position = pos[:,np.newaxis]
        share_vector = np.array([0,0,0,1],dtype = float)[np.newaxis,:]
        r = R.from_quat(quat)
        rotation_matrix = r.as_matrix()
        m34 = np.concatenate((rotation_matrix, position),axis = 1)
        matrix = np.concatenate((m34,share_vector),axis = 0)
        return matrix

if __name__ == "__main__":
    rclpy.init(args=None)
    node = TF_Reader()
    rclpy.spin(node)
    rclpy.shutdown()