#!/usr/bin/env python3
import cv2
import math
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2,CompressedImage
from seojin_pkg.msg import Yolo_bbox_arr, Yolo_bbox
from LiDAR.msg import object_msg_arr
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridgeError

class Fusion: 
    def __init__(self):
        # Lidar
        rospy.Subscriber('/4_Clustering_PCL2', PointCloud2, self.point_cloud_callback)
        rospy.Subscriber('/4_Clustering_OMA', object_msg_arr, self.LidarCallback)

        # Object Detection
        rospy.Subscriber('/YOLO/BBox', Yolo_bbox_arr, self.YoloCallback)    

        # Camera     
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback_Image)

        self.height = 720
        self.width = 1020 
        # fov 70 -> 730
        # fov 90 -> 510
        # fov 130 -> 238
        self.Intrinsic_parameter = np.array([[   510   ,      0    ,    510   ,0],
                                            [     0    ,    510   ,     360   ,0],
                                            [     0    ,      0    ,      1    ,0]],dtype=np.float64)      

        # -t !!                               
        self.translation_matrix=np.array([[1,0,0, 1.1],
                                        [0,1,0,   0 ],
                                        [0,0,1, -0.4],
                                        [0,0,0,   1  ]])
        
        self.FinalMatrix = self.CalculateTransformationMatrix(np.deg2rad(-90.), np.deg2rad(90.), 0.)

        self.st_BGRImg = None
        self.LidarImage = None

        self.objectPoints = []   

        self.u16_ObjCntLidar = 0
        self.ar_LidarObject3d = []     

        self.u16_ObjCntCamera = 0
        self.arst_CamObject = []

        self.arst_CamBBoxes = []
        self.arst_LidarBBoxes = []

        self.rate = rospy.Rate(50) 
        self.WholeProcessiong()


    def callback_Image(self, msg):
        np_arr=np.frombuffer(msg.data,np.uint8)
        img_bgr=cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
        if img_bgr is not None:
            self.st_BGRImg=img_bgr


    def YoloCallback(self, arst_CamObject):
        self.u16_ObjCntCamera = arst_CamObject.objc
        self.arst_CamObject = arst_CamObject.yolo_bbox_arr
        

    def LidarCallback(self, arst_LidarObject):
        self.u16_ObjCntLidar = arst_LidarObject.objc
        self.ar_LidarObject3d = arst_LidarObject.object_msg_arr 


    def point_cloud_callback(self, st_Msg):
        pc_data = pc2.read_points(st_Msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(list(pc_data))
        self.objectPoints = points.astype(np.float32)
        if self.st_BGRImg is not None:
            self.Lidar2Image()

   


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
    
    def LidarBasedIou(self, LidarBBox, CameraBBox):
        if((max(LidarBBox[1][0],CameraBBox.xMax)-min(LidarBBox[0][0],CameraBBox.xMin))>(CameraBBox.xMax - CameraBBox.xMin)+(LidarBBox[1][0] - LidarBBox[0][0]) 
           or (max(LidarBBox[1][1],CameraBBox.yMax)-min(LidarBBox[0][1],CameraBBox.yMin))>(CameraBBox.yMax - CameraBBox.yMin)+(LidarBBox[1][1] - LidarBBox[0][1])):
            IoU = 0
        else:
            Lidarbbox=(LidarBBox[1][0] - LidarBBox[0][0])*(LidarBBox[1][1] - LidarBBox[0][1])

            if Lidarbbox == 0: 
                IoU = 0
            else:
                intersection_xmin = max(CameraBBox.xMin,LidarBBox[0][0])
                intersection_xmax = min(CameraBBox.xMax,LidarBBox[1][0])
                intersection_ymin = max(CameraBBox.yMin,LidarBBox[0][1])
                intersection_ymax = min(CameraBBox.yMax,LidarBBox[1][1])

                intersection_bbox_size = (intersection_xmax-intersection_xmin)*(intersection_ymax-intersection_ymin)
                IoU = intersection_bbox_size / (Lidarbbox)

        return IoU
    
    def Iou(self, LidarBBox, CameraBBox):
        if((max(LidarBBox[1][0],CameraBBox.xMax)-min(LidarBBox[0][0],CameraBBox.xMin))>(CameraBBox.xMax - CameraBBox.xMin)+(LidarBBox[1][0] - LidarBBox[0][0]) 
           or (max(LidarBBox[1][1],CameraBBox.yMax)-min(LidarBBox[0][1],CameraBBox.yMin))>(CameraBBox.yMax - CameraBBox.yMin)+(LidarBBox[1][1] - LidarBBox[0][1])):
            IoU = 0
        else:
            Lidarbbox =(LidarBBox[1][0] - LidarBBox[0][0])*(LidarBBox[1][1] - LidarBBox[0][1]) # 
            Camerabbox = (CameraBBox.xMax - CameraBBox.xMin)*(CameraBBox.yMax - CameraBBox.yMin) # 
            if Lidarbbox == 0 or Camerabbox == 0: 
                IoU = 0
            else:
                intersection_xmin = max(CameraBBox.xMin,LidarBBox[0][0])
                intersection_xmax = min(CameraBBox.xMax,LidarBBox[1][0])
                intersection_ymin = max(CameraBBox.yMin,LidarBBox[0][1])
                intersection_ymax = min(CameraBBox.yMax,LidarBBox[1][1])

                intersection_bbox_size = (intersection_xmax-intersection_xmin)*(intersection_ymax-intersection_ymin)
                IoU = intersection_bbox_size / (Lidarbbox + Camerabbox - intersection_bbox_size)

        return IoU
    

    def Lidar2Image(self):
        
        tmp_LidarImage = self.st_BGRImg.copy()

        #  lidar to camera 회전행렬
        #FinalMatrix = self.CalculateTransformationMatrix(np.deg2rad(-90.), np.deg2rad(90.), 0.)
        #print(len(self.objectPoints))
        ObjPoints = self.objectPoints

        for i in range(len(ObjPoints)):
            if ObjPoints[i][0] > 0 and ObjPoints[i][1] >= -20 and ObjPoints[i][1] <= 20 and ObjPoints[i][2] >= -2 and ObjPoints[i][2] <= 5:
                LidarPosition3d = np.array([[ObjPoints[i][0], ObjPoints[i][1], ObjPoints[i][2], 1]]) 
                Lidar2Camera = np.matmul(self.FinalMatrix, np.transpose(LidarPosition3d))  # 최종 캘리브레이션 결과
                Lidar2Image = [int(Lidar2Camera[0][0] / Lidar2Camera[2][0]), int(Lidar2Camera[1][0] / Lidar2Camera[2][0])] # [X, Y, 1]형태로 만들기 
                if 0 <= Lidar2Image[0] < 1020 and 0 <= Lidar2Image[1] < 720:
                    cv2.circle(tmp_LidarImage, (int(Lidar2Image[0]),int(Lidar2Image[1])), 1, (0,0,255), 2)

        self.LidarImage = tmp_LidarImage

    def check_project_case(self, obj):
        # U = up, M = middle, D = down, L = left, R = right
        # UL = 0, UM = 1, UR = 2, ML = 3, MM = 4, ML = 5, DL = 6, DM = 7, DR = 8
        # x is always positive, each case depends on y,z sign
        
        flag = None
        
        if obj.zMin >= 0 and obj.zMax >= 0:                       # U?
            if obj.yMin >= 0 and obj.yMin >= 0: flag = 0          # UL
            elif obj.yMin <= 0 and obj.yMin >= 0: flag = 1        # UM
            else: flag = 2                                        # UR
        elif obj.zMin <= 0 and obj.zMax >= 0:                     # M?
            if obj.yMin >= 0 and obj.yMax >= 0: flag = 3          # ML
            elif obj.yMin <= 0 and obj.yMax >= 0: flag = 4        # MM
            else: flag = 5                                        # MR
        elif obj.zMin <= 0 and obj.zMax <= 0:                     # D?
            if obj.yMin >= 0 and obj.yMax >= 0: flag = 6          # DL
            elif obj.yMin <= 0 and obj.yMax >= 0: flag = 7        # DM
            else: flag = 8                                        # DR
    
        return flag


    def LidarProcessing(self):
        # transform
        LidarBBoxes = []

        # Lidar BBox
        for i in range(self.u16_ObjCntLidar):
            flag = self.check_project_case(self.ar_LidarObject3d[i])
            # Core point 변환 
            st_LidarPosition3d = np.array([[self.ar_LidarObject3d[i].x, self.ar_LidarObject3d[i].y, self.ar_LidarObject3d[i].z, 1]])  # 동차 행렬 생성
            st_Lidar2Camera = np.matmul(self.FinalMatrix, np.transpose(st_LidarPosition3d))  # 최종 캘리브레이션 결과
            st_Lidar2Image = [int(st_Lidar2Camera[0][0] / st_Lidar2Camera[2][0]), int(st_Lidar2Camera[1][0] / st_Lidar2Camera[2][0])]
            
            
            # if flag == 2 or flag == 6: 
            #     # Max point 변환
            #     st_LidarPosition3d = np.array([[self.ar_LidarObject3d[i].xMax,self.ar_LidarObject3d[i].yMax,self.ar_LidarObject3d[i].zMax,1]])
            #     st_Lidar2Camera = np.matmul(self.FinalMatrix, np.transpose(st_LidarPosition3d))
            #     st_LidarObjectMax = [int(st_Lidar2Camera[0][0] / st_Lidar2Camera[2][0]), int(st_Lidar2Camera[1][0] / st_Lidar2Camera[2][0])]

            #     # Min point 변환 
            #     st_LidarPosition3d = np.array([[self.ar_LidarObject3d[i].xMin,self.ar_LidarObject3d[i].yMin,self.ar_LidarObject3d[i].zMin,1]])
            #     st_Lidar2Camera = np.matmul(self.FinalMatrix, np.transpose(st_LidarPosition3d))
            #     st_LidarObjectMin = [int(st_Lidar2Camera[0][0] / st_Lidar2Camera[2][0]), int(st_Lidar2Camera[1][0] / st_Lidar2Camera[2][0])]
            # else: 
            #     # Max point 변환
            #     st_LidarPosition3d = np.array([[self.ar_LidarObject3d[i].xMin,self.ar_LidarObject3d[i].yMax,self.ar_LidarObject3d[i].zMax,1]])
            #     st_Lidar2Camera = np.matmul(self.FinalMatrix, np.transpose(st_LidarPosition3d))
            #     st_LidarObjectMax = [int(st_Lidar2Camera[0][0] / st_Lidar2Camera[2][0]), int(st_Lidar2Camera[1][0] / st_Lidar2Camera[2][0])]

            #     # Min point 변환 
            #     st_LidarPosition3d = np.array([[self.ar_LidarObject3d[i].xMax,self.ar_LidarObject3d[i].yMin,self.ar_LidarObject3d[i].zMin,1]])
            #     st_Lidar2Camera = np.matmul(self.FinalMatrix, np.transpose(st_LidarPosition3d))
            #     st_LidarObjectMin = [int(st_Lidar2Camera[0][0] / st_Lidar2Camera[2][0]), int(st_Lidar2Camera[1][0] / st_Lidar2Camera[2][0])]

            # ------------------------------------------------------------------------------------------
            # Max point 변환
            st_LidarPosition3d = np.array([[self.ar_LidarObject3d[i].atYmax.x,self.ar_LidarObject3d[i].atYmax.y,self.ar_LidarObject3d[i].atYmax.z,1]])
            st_Lidar2Camera = np.matmul(self.FinalMatrix, np.transpose(st_LidarPosition3d))
            st_LidarObjectMax = [int(st_Lidar2Camera[0][0] / st_Lidar2Camera[2][0]), 0]

            # Min point 변환 
            st_LidarPosition3d = np.array([[self.ar_LidarObject3d[i].atYmin.x,self.ar_LidarObject3d[i].atYmin.y,self.ar_LidarObject3d[i].atYmin.z,1]])
            st_Lidar2Camera = np.matmul(self.FinalMatrix, np.transpose(st_LidarPosition3d))
            st_LidarObjectMin = [int(st_Lidar2Camera[0][0] / st_Lidar2Camera[2][0]), 0]

            # zMax 
            st_LidarPosition3d = np.array([[self.ar_LidarObject3d[i].atZmax.x,self.ar_LidarObject3d[i].atZmax.y,self.ar_LidarObject3d[i].atZmax.z,1]])
            st_Lidar2Camera = np.matmul(self.FinalMatrix, np.transpose(st_LidarPosition3d))
            st_LidarObjectMax[1] = int(st_Lidar2Camera[1][0] / st_Lidar2Camera[2][0])

            # zMin
            st_LidarPosition3d = np.array([[self.ar_LidarObject3d[i].xMin,self.ar_LidarObject3d[i].y,self.ar_LidarObject3d[i].zMin,1]])
            st_Lidar2Camera = np.matmul(self.FinalMatrix, np.transpose(st_LidarPosition3d))
            st_LidarObjectMin[1] = int(st_Lidar2Camera[1][0] / st_Lidar2Camera[2][0]) 

            # core point가 이미지 안에 존재할 경우
            if 0 <= st_LidarObjectMax[0] < self.width or 0 <= st_LidarObjectMin[0] < self.width or  0 <= st_LidarObjectMax[1] < self.height or 0 <= st_LidarObjectMin[1] < self.height:
                if st_LidarObjectMax[0] < 0:
                    st_LidarObjectMax[0] = 2
                if st_LidarObjectMax[1] < 0:
                    st_LidarObjectMax[1] = 2

                if st_LidarObjectMin[0] > self.width:
                    st_LidarObjectMin[0] = self.width-2
                if st_LidarObjectMin[1] > self.height:
                    st_LidarObjectMin[1] = self.height-2
                # cv2.circle(tmp_Image, (int(st_Lidar2Image[0]),int(st_Lidar2Image[1])), 1, (0,0,255), 2)   
                # cv2.circle(tmp_Image, st_LidarObjectMin, 1, (0,255,0), 2)   
                # cv2.circle(tmp_Image, st_LidarObjectMax, 1, (0,0,255), 4)   
                # cv2.rectangle(tmp_Image,st_LidarObjectMin,st_LidarObjectMax,(0,255,0), 2)   
                #cv2.rectangle(self.BBoxImage,self.Min,self.Max,(255,0,0), 2) 
                LidarBBoxes.append([st_LidarObjectMax,st_LidarObjectMin])

        self.arst_LidarBBoxes = LidarBBoxes

    def DrawBBox(self):
        # Camera BBox
        tmp_BBoxImage = self.st_BGRImg.copy()
        CameraBBoxes = []
        tmp_ObjCntCamera = self.u16_ObjCntCamera
        tmp_CamObject = self.arst_CamObject
        for i in range(tmp_ObjCntCamera):
            st_CamObjectMin = [int(tmp_CamObject[i].xMin),int(tmp_CamObject[i].yMin)] 
            st_CamObjectMax = [int(tmp_CamObject[i].xMax),int(tmp_CamObject[i].yMax)] 
            cv2.rectangle(tmp_BBoxImage,st_CamObjectMin,st_CamObjectMax,(0,0,255), 2) 

            CameraBBoxes.append(tmp_CamObject[i]) 

        self.arst_CamBBoxes = CameraBBoxes
        self.BBoxImage = tmp_BBoxImage

        # Lidar BBox
        for idx,LiDarBBox in enumerate(self.arst_LidarBBoxes):
            #cv2.circle(tmp_Image, (int(st_Lidar2Image[0]),int(st_Lidar2Image[1])), 1, (0,0,255), 2)   
            cv2.circle(tmp_BBoxImage, LiDarBBox[1], 1, (0,255,0), 2)   
            cv2.circle(tmp_BBoxImage, LiDarBBox[0], 1, (0,0,255), 4)   
            cv2.rectangle(tmp_BBoxImage,LiDarBBox[1],LiDarBBox[0],(0,255,0), 2)  

        self.BBoxImage = tmp_BBoxImage

    def FusionProcessing(self):
        # cal iou 
        tmp_BBoxImage = self.BBoxImage.copy()
        for CameraIndex, CameraBBox in enumerate(self.arst_CamBBoxes): 

            for LiDarIndex, LiDarBBox in enumerate(self.arst_LidarBBoxes):

                IoU = self.Iou(LiDarBBox, CameraBBox) 
                #lidar BBox 가 화면을 넘어갈 경우 예외 처리 필요 
                print(IoU)
                if IoU > 0.1: 
                    distance = math.sqrt(self.ar_LidarObject3d[LiDarIndex].x**2 + self.ar_LidarObject3d[LiDarIndex].y**2)
                    cv2.putText(
                        tmp_BBoxImage,
                        f"{IoU:.2f}", 
                        (LiDarBBox[0][0], LiDarBBox[0][1] - 20),  # y1-20으로 위로 조금 이동
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 0, 0),
                        2
                    )
                    distance = math.sqrt(self.ar_LidarObject3d[LiDarIndex].x**2 + self.ar_LidarObject3d[LiDarIndex].y**2)
                    print(distance)
                    #   distance , index, info 
        self.BBoxImage  = tmp_BBoxImage
 
    def WholeProcessiong(self):

        while self.st_BGRImg is None or self.LidarImage is None:
            pass
        
        while not rospy.is_shutdown():
            self.LidarProcessing()
            self.DrawBBox()
            self.FusionProcessing() 
            cv2.imshow("Lidar2Image",self.LidarImage)
            cv2.imshow("BBox", self.BBoxImage)
            cv2.waitKey(1)
            self.rate.sleep()


if __name__ =='__main__':
    rospy.init_node('lidar_projection',anonymous=True)
    Fusion()
    rospy.spin()