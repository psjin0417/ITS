#!/usr/bin/env python3
import cv2
import math
import rospy
import numpy as np
from sensor_msgs.msg import Imu,CompressedImage
from camera_ITS.msg import Yolo_object_arr, object, Coordinate
import tf.transformations as tf_trans
from cv_bridge import CvBridgeError
from collections import deque
import copy
from ultralytics import YOLO
import sympy as sp

class Fusion: 
    def __init__(self):

        # Camera     
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback_Image)
        rospy.Subscriber("/Cam/right", CompressedImage, self.callback_right_Image)
        rospy.Subscriber("/Cam/left", CompressedImage, self.callback_left_Image)
        rospy.Subscriber("/imu", Imu, self.callback_Imu)
        rospy.Subscriber("/YOLO/Object", Yolo_object_arr, self.callback_YOLO)

        self.height = 720
        self.width = 1020 

        # IPM option

        # [front, left, right]
        world_x_max = [20, 13, 13]
        world_x_min = [4, -1, -1]
        world_y_max = [8, 8, -1]
        world_y_min = [-8, 1, -8]

        self.world_pts = []

        for i in range(3):

            arr_world_pts = np.float32([[world_x_max[i], world_y_max[i], 0, 1],
                         [world_x_min[i], world_y_max[i], 0, 1],
                         [world_x_max[i], world_y_min[i], 0, 1],
                         [world_x_min[i], world_y_min[i], 0, 1]])
            self.world_pts.append(arr_world_pts)

        ## 
        Intrinsic_parameter_90 = np.array([[   428   ,      0    ,    510   ,0],
                                            [     0    ,    428   ,     360   ,0],
                                            [     0    ,      0    ,      1    ,0]],dtype=np.float64)      
        Intrinsic_parameter_130 = np.array([[   238   ,      0    ,    510   ,0],
                                            [     0    ,    238   ,     360   ,0],
                                            [     0    ,      0    ,      1    ,0]],dtype=np.float64)      

        # top cam                                
        translation_matrix_Fcam = np.array([[1,0,0, -3.6],
                                        [0,1,0,   0 ],
                                        [0,0,1, -1.0],
                                        [0,0,0,   1  ]])

        translation_matrix_Lcam = np.array([[1,0,0, -2.3],
                                        [0,1,0,  -1.0],
                                        [0,0,1, -1.0],
                                        [0,0,0,   1  ]])
        
        translation_matrix_Rcam = np.array([[1,0,0, -2.3],
                                        [0,1,0,  1.0],
                                        [0,0,1, -1.0],
                                        [0,0,0,   1  ]])
        
        
        FinalMatrix_left = self.CalculateTransformationMatrix(np.deg2rad(-135.), 0, 0, Intrinsic_parameter_130, translation_matrix_Lcam)
        FinalMatrix_right = self.CalculateTransformationMatrix(np.deg2rad(-45.), np.deg2rad(180.), 0, Intrinsic_parameter_130, translation_matrix_Rcam)
        FinalMatrix_front = self.CalculateTransformationMatrix(0, np.deg2rad(110.), np.deg2rad(-90.), Intrinsic_parameter_90, translation_matrix_Fcam)

        self.FinalMatrix = [FinalMatrix_front, FinalMatrix_left, FinalMatrix_right]

        # 변환 후 좌표(image)
        self.dst_height = 600
        self.dst_width = 300
        
        self.pts2 = np.float32([[0,0], [0,self.dst_height], [self.dst_width,0], [self.dst_width,self.dst_height]])
        self.arr_pts2 = [np.float32([[0,0], [0,686], [686,0], [686,686]]),self.pts2,self.pts2]

       
        # canvus
        self.canvas_height, self.canvas_width = 900, 700

        ## IMU 
        self.init_pitch = None
        self.init_roll = None
        self.yolo = None

        self.N = 2
        self.front_queue = deque(maxlen=self.N)
        self.left_queue = deque(maxlen=self.N)
        self.right_queue = deque(maxlen=self.N)

        self.st_BGRImg = None
        self.right_dst_cal = None
        self.left_dst_cal = None
        self.front_dst_cal = None

        # img 
        car_img = cv2.imread('./src/camera_ITS/src/car.png')
        self.car_img = cv2.resize(car_img, (115, 230))
        h, w = self.car_img.shape[:2]
        alpha_channel = np.ones((h, w), dtype=np.uint8) * 255  # 255 = 완전 불투명
        threshold = 1  # 어두운 색 기준값
        black_mask = (self.car_img[:, :, 0] < threshold) & (self.car_img[:, :, 1] < threshold) & (self.car_img[:, :, 2] < threshold)
        alpha_channel[black_mask] = 0  # 알파값 0으로 변경 (투명)
        self.car_img = cv2.merge((self.car_img, alpha_channel))


        self.rate = rospy.Rate(30) 
        self.WholeProcessiong()


    def callback_Image(self, msg):
        np_arr=np.frombuffer(msg.data,np.uint8)
        img_bgr=cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
        if img_bgr is not None and self.init_pitch is not None:
            pitch = self.pitch
            roll = self.roll
            self.st_BGRImg=img_bgr
            self.front_dst = self.gen_BEV_origin(0,img_bgr, 686, 686, pitch, roll )
            self.front_dst_cal = self.gen_BEV(0,img_bgr, 686, 686, pitch, roll )
            
    def callback_right_Image(self, msg):
        
        np_arr=np.frombuffer(msg.data,np.uint8)
        img_bgr=cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
        if img_bgr is not None and self.init_pitch is not None:
            pitch = self.pitch
            roll = self.roll
            self.st_right_Img=img_bgr
            self.right_dst_cal = self.gen_BEV(2,img_bgr, self.dst_width, self.dst_height, pitch, roll )
            self.right_dst = self.gen_BEV_origin(2,img_bgr, self.dst_width, self.dst_height, pitch, roll )


    def callback_left_Image(self, msg):
        np_arr=np.frombuffer(msg.data,np.uint8)
        img_bgr=cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
        if img_bgr is not None and self.init_pitch is not None:
            pitch = self.pitch
            roll = self.roll
            self.st_left_Img=img_bgr
            self.left_dst_cal = self.gen_BEV(1,img_bgr, self.dst_width, self.dst_height, pitch, roll )
            self.left_dst= self.gen_BEV_origin(1,img_bgr, self.dst_width, self.dst_height, pitch, roll )

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

    def callback_YOLO(self, msg):
        for obj in msg.data:
            self.cal_dist(obj.CamNum, obj.BB.x, obj.BB.y)

    def image_to_world(self, K, R, t, uv):
        # 이미지 좌표 (u, v)
        u, v = uv

        # 카메라 내부 행렬 K의 역행렬
        K_inv = np.linalg.inv(K)

        # 이미지 좌표를 정규화된 카메라 좌표로 변환
        pixel_homogeneous = np.array([u, v, 1])
        cam_normalized = np.dot(K_inv, pixel_homogeneous)

        # 월드 좌표 구하기 (Z_w = 0에서)
        R_inv = np.linalg.inv(R)
        t_vec = np.dot(R_inv, t)  # R^-1 * t

        # 깊이 값 d를 계산 (Z_w = 0을 만족하도록)
        d = t_vec[2] / (np.dot(R_inv[2, :], cam_normalized))

        # 월드 좌표 X_w, Y_w 계산
        world_point = np.dot(R_inv, cam_normalized * d - t_vec)
        world_point[2] = 0  # Z_w = 0

        return world_point[:3]  # (X_w, Y_w, 0)



    def CalculateTransformationMatrix(self, roll, pitch, yaw, Intrinsic_parameter, translation_matrix):

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

        self.R_P_t = R_P

        R_P = np.array([[R_P[0][0],R_P[0][1],R_P[0][2],0],
                        [R_P[1][0],R_P[1][1],R_P[1][2],0],
                        [R_P[2][0],R_P[2][1],R_P[2][2],0],
                        [0,0,0,1]])
        # 점고정 좌표계 변환을 위한 transpose 
        
        
        return np.matmul(np.matmul(Intrinsic_parameter, R_P.T), translation_matrix)
    
    def MakeZero(self, arr):
        arr[abs(arr)<=6.123234e-16] = 0
        return arr
    
    def gen_BEV(self, i, img, width, height, pitch, roll):
        
        curr_z1 = (self.world_pts[i][0][0]*math.tan(pitch - self.init_pitch)) - self.world_pts[i][0][1]*math.tan(roll - self.init_roll)
        curr_z2 = (self.world_pts[i][1][0]*math.tan(pitch - self.init_pitch)) - self.world_pts[i][1][1]*math.tan(roll - self.init_roll)
        curr_z3 = (self.world_pts[i][2][0]*math.tan(pitch - self.init_pitch)) - self.world_pts[i][2][1]*math.tan(roll - self.init_roll)
        curr_z4 = (self.world_pts[i][3][0]*math.tan(pitch - self.init_pitch)) - self.world_pts[i][3][1]*math.tan(roll - self.init_roll)

        # MAF적용
        if i == 0:
            self.front_queue.appendleft([curr_z1,curr_z2,curr_z3,curr_z4])
            queue = self.front_queue
        elif i == 1:
            self.left_queue.appendleft([curr_z1,curr_z2,curr_z3,curr_z4])
            queue = self.left_queue
        else:
            self.right_queue.appendleft([curr_z1,curr_z2,curr_z3,curr_z4])
            queue = self.right_queue

        tmp_world_pts = copy.deepcopy(self.world_pts)
        tmp_world_pts[i][0][2] = round(sum(z[0] for z in queue) / len(queue), 4)
        tmp_world_pts[i][1][2] = round(sum(z[1] for z in queue) / len(queue), 4)
        tmp_world_pts[i][2][2] = round(sum(z[2] for z in queue) / len(queue), 4)
        tmp_world_pts[i][3][2] = round(sum(z[3] for z in queue) / len(queue), 4)


        pts1 = []
        for k,pt in enumerate(tmp_world_pts[i]):
            world2Camera = np.matmul(self.FinalMatrix[i], np.transpose(tmp_world_pts[i][k])) 
            world2Image = [np.float32(world2Camera[0] / world2Camera[2]), np.float32(world2Camera[1] / world2Camera[2])]
            pts1.append(world2Image)

        pts1 = np.float32(pts1)
        perspective_mtrx = cv2.getPerspectiveTransform(pts1, self.arr_pts2[i])

        return cv2.warpPerspective(img, perspective_mtrx, (width,height))
    
    def gen_BEV_origin(self, i, img, width, height, pitch, roll):
        A_inv = np.linalg.pinv(self.FinalMatrix[i])
        pts1 = []
        for k,pt in enumerate(self.world_pts[i]):
            world2Camera = np.matmul(self.FinalMatrix[i], np.transpose(self.world_pts[i][k])) 
            world2Image = [np.float32(world2Camera[0] / world2Camera[2]), np.float32(world2Camera[1] / world2Camera[2])]
            pts1.append(world2Image)

        pts1 = np.float32(pts1)
        perspective_mtrx = cv2.getPerspectiveTransform(pts1, self.arr_pts2[i])

        return cv2.warpPerspective(img, perspective_mtrx, (width,height))
    
    def cal_dist(self, i,image_x, image_y):
        
        x, y = sp.symbols('x y')
        z = (x*math.tan(self.pitch - self.init_pitch)) - y*math.tan(self.roll - self.init_roll)
        world_pt = sp.Matrix([[x, y, z, 1]]) 
        FinalMatrix_sp = sp.Matrix(self.FinalMatrix[i])
        world2Camera = FinalMatrix_sp * world_pt.T

        eq1 = sp.Eq(world2Camera[0] / world2Camera[2], image_x)
        eq2 = sp.Eq(world2Camera[1] / world2Camera[2], image_y)
        solution = sp.solve((eq1, eq2), (x, y))
        distance = sp.sqrt(solution[x]**2 + solution[y]**2)
        print(" 거리:", distance)

    def WholeProcessiong(self):

        while self.front_dst_cal is None or self.left_dst_cal is None or self.right_dst_cal is None:
            pass

        while not rospy.is_shutdown():


            img = self.st_BGRImg.copy()
            tmp_front_dst_cal = self.front_dst_cal.copy()
            tmp_left_dst_cal = self.left_dst_cal.copy()
            tmp_right_dst_cal = self.right_dst_cal.copy()

            # 알파채널 
            h, w = tmp_front_dst_cal.shape[:2]
            alpha_channel = np.ones((h, w), dtype=np.uint8) * 255  # 255 = 완전 불투명
            threshold = 1  # 어두운 색 기준값
            black_mask = (tmp_front_dst_cal[:, :, 0] < threshold) & (tmp_front_dst_cal[:, :, 1] < threshold) & (tmp_front_dst_cal[:, :, 2] < threshold)
            alpha_channel[black_mask] = 0  # 알파값 0으로 변경 (투명)
            rgba_image = cv2.merge((tmp_front_dst_cal, alpha_channel))

        

            positions = [(300, 2), (300, 384), (0,0),(670,288)]
            
            # 보정O
            canvas1 = np.zeros((self.canvas_height, self.canvas_width, 3), dtype=np.uint8)
            idx = 0
            for img, (y, x) in zip([tmp_left_dst_cal, tmp_right_dst_cal, rgba_image, self.car_img], positions):
                idx += 1
                h, w, _ = img.shape
                #print(y,x,h,w)
                if idx == 3 or idx == 4:
                    #canvas[y:y+h, x:x+w] = cv2.addWeighted(canvas[y:y+h, x:x+w], 0.8, img, 0.2, 0)
                    #canvas1[y:y+h, x:x+w] = cv2.bitwise_or(canvas1[y:y+h, x:x+w], img)  # 해당 위치에 이미지 배치
                    #canvas = np.clip(canvas, 0, 255)
                    bgr_image = img[:, :, :3]  # RGB 채널
                    alpha_channel = img[:, :, 3]  # 알파 채널
                    alpha_mask = alpha_channel != 0
                    canvas1[y:y+h, x:x+w][alpha_mask] = bgr_image[alpha_mask]
                else:
                    canvas1[y:y+h, x:x+w] = img
            #cv2.rectangle(canvas1,(288,669), (400,900),(255, 255, 255), thickness=-1)
            canvas1 = canvas1[0:840, 0:650]

            # 보정x 
            canvas2 = np.zeros((self.canvas_height, self.canvas_width, 3), dtype=np.uint8)
            idx = 0
            for img, (y, x) in zip([self.left_dst, self.right_dst, self.front_dst], positions):
                idx += 1
                h, w, _ = img.shape
                #print(y,x,h,w)
                if idx == 3:
                    #canvas[y:y+h, x:x+w] = cv2.addWeighted(canvas[y:y+h, x:x+w], 0.8, img, 0.2, 0)
                    canvas2[y:y+h, x:x+w] = cv2.bitwise_or(canvas2[y:y+h, x:x+w], img)  # 해당 위치에 이미지 배치
                    canvas2 = np.clip(canvas2, 0, 250)
                else:
                    canvas2[y:y+h, x:x+w] = img
                
            cv2.imshow('Merged Image', canvas1)
            cv2.imshow('Merged Image no cal', canvas2)

            cv2.waitKey(1)
            self.rate.sleep()
        
        

if __name__ =='__main__':
    rospy.init_node('lidar_projection',anonymous=True)
    Fusion()
    rospy.spin()