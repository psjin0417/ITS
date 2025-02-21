#!/usr/bin/env python3
import cv2
import math
import rospy
import numpy as np
from sensor_msgs.msg import Imu,CompressedImage
import tf.transformations as tf_trans
from cv_bridge import CvBridgeError
from collections import deque

class Fusion: 
    def __init__(self):

        # Camera     
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback_Image)
        rospy.Subscriber("/imu", Imu, self.callback_Imu)

        self.height = 720
        self.width = 1020 

        # IPM option
        self.world_x_max = 50
        self.world_x_min = 2 
        self.world_y_max = 7
        self.world_y_min = -7

        self.world_pts = np.float32([[self.world_x_max, self.world_y_max, 0, 1],
                         [self.world_x_min, self.world_y_max, 0, 1],
                         [self.world_x_max, self.world_y_min, 0, 1],
                         [self.world_x_min, self.world_y_min, 0, 1]])

        ## 
        self.Intrinsic_parameter = np.array([[   730   ,      0    ,    510   ,0],
                                            [     0    ,    730   ,     360   ,0],
                                            [     0    ,      0    ,      1    ,0]],dtype=np.float64)      

        # -t !!                               
        self.translation_matrix=np.array([[1,0,0, 0],
                                        [0,1,0,   0 ],
                                        [0,0,1, -1.3],
                                        [0,0,0,   1  ]])
        
        self.FinalMatrix = self.CalculateTransformationMatrix(np.deg2rad(-90.), np.deg2rad(90.), 0.)

        # 변환 후 좌표(image)
        self.dst_height = 600
        self.dst_width = 300
        self.pts1 = np.float32([[0,0], [0,self.dst_height], [self.dst_width,0], [self.dst_width,self.dst_height]])
        # 변환 전 좌표(image)
        self.pts2 = []
        for i,pt in enumerate(self.world_pts):
            world2Camera = np.matmul(self.FinalMatrix, np.transpose(self.world_pts[i])) 
            world2Image = [np.float32(world2Camera[0] / world2Camera[2]), np.float32(world2Camera[1] / world2Camera[2])]
            self.pts2.append(world2Image)

        self.pts2 = np.float32(self.pts2)
        self.perspective_mtrx = cv2.getPerspectiveTransform(self.pts2, self.pts1)

        ## IMU 
        self.init_pitch = None
        self.init_roll = None

        self.N = 3
        self.UL_queue = deque(maxlen=self.N)
        self.DL_queue = deque(maxlen=self.N)
        self.UR_queue = deque(maxlen=self.N)
        self.DR_queue = deque(maxlen=self.N)


        self.st_BGRImg = None

        self.rate = rospy.Rate(30) 
        #self.WholeProcessiong()


    def callback_Image(self, msg):
        np_arr=np.frombuffer(msg.data,np.uint8)
        img_bgr=cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
        if img_bgr is not None and self.init_pitch is not None:
            self.st_BGRImg=img_bgr
            self.tmp_pitch = self.pitch 
            self.tmp_roll = self.roll 
            self.WholeProcessiong()

    def callback_Imu(self, msg):
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        roll, pitch, yaw = tf_trans.euler_from_quaternion(quaternion)
        if self.init_pitch == None:
            self.init_pitch = pitch
        if self.init_roll == None:
            self.init_roll = roll
        self.pitch = pitch
        self.roll = roll
        print(roll)
        #print(22*abs(math.tan(self.pitch - self.init_pitch))) 
        # 최초의 PITCH값 저장해서 영점 조절 필요 

    def CalculateTransformationMatrix(self, roll, pitch, yaw):

        Rotation_x = np.array([[     1    ,      0        ,        0       ],
                                [     0    , math.cos(roll), -math.sin(roll)],
                                [     0    , math.sin(roll),  math.cos(roll)]])
        
        Rotation_y = np.array([[ math.cos(pitch) ,  0  ,  math.sin(pitch)  ],
                                [        0        ,  1  ,        0          ],
                                [-math.sin(pitch) ,  0  ,  math.cos(pitch)  ]])
        
        Rotation_z = np.array([[math.cos(yaw), -math.sin(yaw) ,  0   ],
                                [math.sin(yaw),  math.cos(yaw) ,  0   ],
                                [     0       ,       0        ,  1   ]])
        
        R_P = np.matmul(np.matmul(Rotation_x,Rotation_y),Rotation_z)

        R_P = self.MakeZero(R_P)

        R_P = np.array([[R_P[0][0],R_P[0][1],R_P[0][2],0],
                        [R_P[1][0],R_P[1][1],R_P[1][2],0],
                        [R_P[2][0],R_P[2][1],R_P[2][2],0],
                        [0,0,0,1]])
        # 점고정 좌표계 변환을 위한 transpose 
        R_P_t = R_P.T
        
        return np.matmul(np.matmul(self.Intrinsic_parameter, R_P.T), self.translation_matrix)
    
    def MakeZero(self, arr):
        arr[abs(arr)<=6.123234e-16] = 0
        return arr

 
    def WholeProcessiong(self):

        # while self.st_BGRImg is None:
        #     pass

        # while not rospy.is_shutdown():
        img1 = self.st_BGRImg.copy()
        img2 = self.st_BGRImg.copy()
        dst = cv2.warpPerspective(img1, self.perspective_mtrx, (self.dst_width,self.dst_height))


        # imu calibration
        # 음수 
        curr_z1 = self.world_pts[0][0]*math.tan(self.tmp_pitch - self.init_pitch) - self.world_pts[0][1]*math.tan(self.tmp_roll - self.init_roll)
        curr_z2 = self.world_pts[1][0]*math.tan(self.tmp_pitch - self.init_pitch) - self.world_pts[1][1]*math.tan(self.tmp_roll - self.init_roll)
        curr_z3 = self.world_pts[2][0]*math.tan(self.tmp_pitch - self.init_pitch) - self.world_pts[2][1]*math.tan(self.tmp_roll - self.init_roll)
        curr_z4 = self.world_pts[3][0]*math.tan(self.tmp_pitch - self.init_pitch) - self.world_pts[3][1]*math.tan(self.tmp_roll - self.init_roll)

        self.UL_queue.appendleft(curr_z1)
        self.DL_queue.appendleft(curr_z2)
        self.UR_queue.appendleft(curr_z3)
        self.DR_queue.appendleft(curr_z4)

        self.world_pts[0][2] = round(sum(z for z in self.UL_queue) / len(self.UL_queue), 4)
        self.world_pts[1][2] = round(sum(z for z in self.DL_queue) / len(self.DL_queue), 4)
        self.world_pts[2][2] = round(sum(z for z in self.UR_queue) / len(self.UR_queue), 4)
        self.world_pts[3][2] = round(sum(z for z in self.DR_queue) / len(self.DR_queue), 4)

        #print(self.world_pts[0,2])

        self.pts2 = []
        for i,pt in enumerate(self.world_pts):
            world2Camera = np.matmul(self.FinalMatrix, np.transpose(self.world_pts[i])) 
            world2Image = [np.float32(world2Camera[0] / world2Camera[2]), np.float32(world2Camera[1] / world2Camera[2])]
            self.pts2.append(world2Image)

        self.pts2 = np.float32(self.pts2)
        perspective_mtrx = cv2.getPerspectiveTransform(self.pts2, self.pts1)
        dst_cal = cv2.warpPerspective(img2, perspective_mtrx, (self.dst_width,self.dst_height))


        cv2.imshow("IPM with out cal", dst)
        cv2.imshow("IPM with cal", dst_cal)
        cv2.imshow("img", self.st_BGRImg)

        cv2.waitKey(1)
            #self.rate.sleep()
        
        

if __name__ =='__main__':
    rospy.init_node('lidar_projection',anonymous=True)
    Fusion()
    rospy.spin()