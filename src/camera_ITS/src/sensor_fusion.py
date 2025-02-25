#!/usr/bin/env python3
import cv2
import math
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2,CompressedImage
from camera_ITS.msg import Yolo_bbox_arr, Yolo_bbox
from LiDAR.msg import object_msg_arr
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridgeError

class Fusion: 
    def __init__(self):
        # Lidar
        rospy.Subscriber('/4_Clustering_PCL2', PointCloud2, self.point_cloud_callback) #포인트 클라우드 데이터
        rospy.Subscriber('/4_Clustering_OMA', object_msg_arr, self.LidarCallback) #Lidar 객체 데이터

        # Object Detection
        rospy.Subscriber('/YOLO/BBox', Yolo_bbox_arr, self.YoloCallback)    #욜로 객체 탐지 구독

        # Camera     
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback_Image) #카메라 이미지 구독

        self.height = 720
        self.width = 1020 
        # fov 70 -> 730
        # fov 90 -> 510
        # fov 130 -> 238


        #카메라 내적 행렬 정의 (카메라의 초점 거리, 중심 좌표 등)
        self.Intrinsic_parameter = np.array([[   510   ,      0    ,    510   ,0],
                                            [     0    ,    510   ,     360   ,0],
                                            [     0    ,      0    ,      1    ,0]],dtype=np.float64)     

        # -t !!        
        # 라이다 좌표계를 카메라 좌표계로 변한하는 변환 행렬(Translation matrix)                       
        self.translation_matrix=np.array([[1,0,0, 1.1],
                                        [0,1,0,   0 ],
                                        [0,0,1, -0.4],
                                        [0,0,0,   1  ]])
        
        #라이다에서 카메로로의 좌표 변환을 위한 회전 행렬 계산 (roll= -90도, pitch=90도, yaw=0도)
        self.FinalMatrix = self.CalculateTransformationMatrix(np.deg2rad(-90.), np.deg2rad(90.), 0.)

        #카메라 이미지 및 라이다 이미지 저장 변수 초기화
        self.st_BGRImg = None
        self.LidarImage = None

        #라이다 포인트 데이터 저장 리스트
        self.objectPoints = []   

        #라이다 객체 개수 및 데이터 저장 리스트
        self.u16_ObjCntLidar = 0
        self.ar_LidarObject3d = []     

        #카메라 객체 개수 및 데이터 저장 리스트
        self.u16_ObjCntCamera = 0
        self.arst_CamObject = []

        #라이다와 카메라 바운딩 박스 저장 리스트
        self.arst_CamBBoxes = []
        self.arst_LidarBBoxes = []

        #실행 주기 설정
        self.rate = rospy.Rate(50) 
        self.WholeProcessiong()


    def callback_Image(self, msg): #카메라 이미지 콜백 함수
        np_arr=np.frombuffer(msg.data,np.uint8) #ros메시지를 numpy 배열로 변환
        img_bgr=cv2.imdecode(np_arr,cv2.IMREAD_COLOR) #암축된 이미지를 opencv 형식으로 변환
        if img_bgr is not None:
            self.st_BGRImg=img_bgr  #변한된 이미지를 클래스 변수에 저장


    def YoloCallback(self, arst_CamObject): #yolo 객체 탐지 콜백 함수
        self.u16_ObjCntCamera = arst_CamObject.objc #yolo 객체 개수 저장
        self.arst_CamObject = arst_CamObject.yolo_bbox_arr #바운딩 박스 리스트 저장
        

    def LidarCallback(self, arst_LidarObject): #라이다 객체 탐지 콜백 함수
        self.u16_ObjCntLidar = arst_LidarObject.objc  #라이다 객체 개수 저장
        self.ar_LidarObject3d = arst_LidarObject.object_msg_arr #바운딩 박스 리스트 저장


    def point_cloud_callback(self, st_Msg):  #라이다 포인트 클라우드 콜백 함수
        pc_data = pc2.read_points(st_Msg, field_names=("x", "y", "z"), skip_nans=True)  #3d 포인트 읽기
        points = np.array(list(pc_data)) #numpy 배열로 변환
        self.objectPoints = points.astype(np.float32) #포인트를 클래스 변수에 저장
        if self.st_BGRImg is not None: #이미지가 존재하면 라이다 데이터를 이미지에 투영
            self.Lidar2Image()

   


    def CalculateTransformationMatrix(self, roll, pitch, yaw):
        #x축 회전 행렬 (roll)
        Rotation_x = np.array([[     1    ,      0        ,        0       ],
                                [     0    , math.cos(roll), -math.sin(roll)],
                                [     0    , math.sin(roll),  math.cos(roll)]])
        
        #y축 회전 행렬(pitch)
        Rotation_y = np.array([[ math.cos(pitch) ,  0  ,  math.sin(pitch)  ],
                                [        0        ,  1  ,        0          ],
                                [-math.sin(pitch) ,  0  ,  math.cos(pitch)  ]])
        
        #z축 회전 행렬(yaw)
        Rotation_z = np.array([[math.cos(yaw), -math.sin(yaw) ,  0   ],
                                [math.sin(yaw),  math.cos(yaw) ,  0   ],
                                [     0       ,       0        ,  1   ]])
        
        #최종 회전 행렬 (x,y,z 순으로 곱함)
        R_P = np.matmul(np.matmul(Rotation_x,Rotation_y),Rotation_z)

        #매우 작은 값을 0으로 처리
        R_P = self.MakeZero(R_P)

        #4x4 변환 행렬 생성
        R_P = np.array([[R_P[0][0],R_P[0][1],R_P[0][2],0],
                        [R_P[1][0],R_P[1][1],R_P[1][2],0],
                        [R_P[2][0],R_P[2][1],R_P[2][2],0],
                        [0,0,0,1]])
        # 점고정 좌표계 변환을 위한 transpose 
        R_P_t = R_P.T
        
        #최종 변환 행렬 계산
        return np.matmul(np.matmul(self.Intrinsic_parameter, R_P.T), self.translation_matrix)
    

    #불필요한 작은 값은 0으로 만들기
    def MakeZero(self, arr):
        arr[abs(arr)<=6.123234e-16] = 0
        return arr
    
    #라이다 기반 iou 계산
    def LidarBasedIou(self, LidarBBox, CameraBBox): #라이다 바운딩 박스와 카메라 바운딩 박스 간의 iou 계산
        #두 박스가 아예 겹치지 않는 경우 두가지: 두박스의 가로 길이 합이랑 비교, 두박스의 세로 길이 합이랑 비교
        if((max(LidarBBox[1][0],CameraBBox.xMax)-min(LidarBBox[0][0],CameraBBox.xMin))>(CameraBBox.xMax - CameraBBox.xMin)+(LidarBBox[1][0] - LidarBBox[0][0]) 
           or (max(LidarBBox[1][1],CameraBBox.yMax)-min(LidarBBox[0][1],CameraBBox.yMin))>(CameraBBox.yMax - CameraBBox.yMin)+(LidarBBox[1][1] - LidarBBox[0][1])):
            IoU = 0 #겹치지 않다고 판단하여 iou=0
        else:
            Lidarbbox=(LidarBBox[1][0] - LidarBBox[0][0])*(LidarBBox[1][1] - LidarBBox[0][1]) #라이다 바운딩 박스 면적 계산 (가로*세로)

            if Lidarbbox == 0: #라이다 바운딩 박스 크기 0이면 iou또한 0
                IoU = 0
            else:
                intersection_xmin = max(CameraBBox.xMin,LidarBBox[0][0]) #겹치는 영역의 왼쪽 x좌표
                intersection_xmax = min(CameraBBox.xMax,LidarBBox[1][0]) #겹치는 영역의 오른쪽 x좌표
                intersection_ymin = max(CameraBBox.yMin,LidarBBox[0][1]) #겹치는 영역의 위쪽 y좌표
                intersection_ymax = min(CameraBBox.yMax,LidarBBox[1][1]) #겹치는 영역의 아래쪽 y좌표

                #IOU 계산 공식(교차 영역 크기/라이다 박스 크기)
                intersection_bbox_size = (intersection_xmax-intersection_xmin)*(intersection_ymax-intersection_ymin)
                IoU = intersection_bbox_size / (Lidarbbox)

        return IoU
    
    #전체 iou 계산
    def Iou(self, LidarBBox, CameraBBox): #라이다 박스와 카메라 박스의 iou계산
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
    

    def Lidar2Image(self): #라이다 데이터를 카메라 이미지로 변환
        
        tmp_LidarImage = self.st_BGRImg.copy() 

        #  lidar to camera 회전행렬
        #FinalMatrix = self.CalculateTransformationMatrix(np.deg2rad(-90.), np.deg2rad(90.), 0.)
        #print(len(self.objectPoints))

        ObjPoints = self.objectPoints #라이다 포인트 데이터 가져오기

        for i in range(len(ObjPoints)): #모든 라이다 포인트에 대해 변환 수행
            if ObjPoints[i][0] > 0 and ObjPoints[i][1] >= -20 and ObjPoints[i][1] <= 20 and ObjPoints[i][2] >= -2 and ObjPoints[i][2] <= 5:
                #ObjPoints[i][0] = X , ObjPoints[i][1] = Y, ObjPoints[i][2] = Z
                #x>0: 라이다에서 앞쪽의 데이터만 사용 / -20<y<20: 차량 주변의 포인트만 사용, -2<z<5: 도로와 차량 높이에 해당하는 포인트만 사용
                LidarPosition3d = np.array([[ObjPoints[i][0], ObjPoints[i][1], ObjPoints[i][2], 1]]) #라이다 포인트를 4D 동차 좌표로 변환
                Lidar2Camera = np.matmul(self.FinalMatrix, np.transpose(LidarPosition3d))  # 최종 캘리브레이션 결과(라이다 포인트를 카메라 좌표계로 변환)
                Lidar2Image = [int(Lidar2Camera[0][0] / Lidar2Camera[2][0]), int(Lidar2Camera[1][0] / Lidar2Camera[2][0])] # [X, Y, 1]형태로 만들기(카메라 좌표계에서 2D이미지로 좌표 변환)
                if 0 <= Lidar2Image[0] < 1020 and 0 <= Lidar2Image[1] < 720:
                    cv2.circle(tmp_LidarImage, (int(Lidar2Image[0]),int(Lidar2Image[1])), 1, (0,0,255), 2) #유효한 범위 내에 이쓴 경우 빨간 점으로 표시

        self.LidarImage = tmp_LidarImage #라이다 포인트가 포함된 이미지를 저장

    # def check_project_case(self, obj):
    #     # U = up, M = middle, D = down, L = left, R = right
    #     # UL = 0, UM = 1, UR = 2, ML = 3, MM = 4, ML = 5, DL = 6, DM = 7, DR = 8
    #     # x is always positive, each case depends on y,z sign
        
    #     flag = None
        
    #     if obj.zMin >= 0 and obj.zMax >= 0:                       # U?
    #         if obj.yMin >= 0 and obj.yMin >= 0: flag = 0          # UL
    #         elif obj.yMin <= 0 and obj.yMin >= 0: flag = 1        # UM
    #         else: flag = 2                                        # UR
    #     elif obj.zMin <= 0 and obj.zMax >= 0:                     # M?
    #         if obj.yMin >= 0 and obj.yMax >= 0: flag = 3          # ML
    #         elif obj.yMin <= 0 and obj.yMax >= 0: flag = 4        # MM
    #         else: flag = 5                                        # MR
    #     elif obj.zMin <= 0 and obj.zMax <= 0:                     # D?
    #         if obj.yMin >= 0 and obj.yMax >= 0: flag = 6          # DL
    #         elif obj.yMin <= 0 and obj.yMax >= 0: flag = 7        # DM
    #         else: flag = 8                                        # DR
    
    #     return flag


    def LidarProcessing(self):
        # 라이다 데이터를 이미지 좌표로 변환하고 바운딩 박스 생성
        LidarBBoxes = []

        # Lidar BBox
        for i in range(self.u16_ObjCntLidar): #감지된 라이다 객체 수만큼 반복
            flag = self.check_project_case(self.ar_LidarObject3d[i]) #특정 조건을 확인하는 함수 호출

            # Core point 변환 (라이다 좌표를 카메라 좌표로 변환)
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

            # Max point 변환(최대 y값 지점 변환)
            st_LidarPosition3d = np.array([[self.ar_LidarObject3d[i].atYmax.x,self.ar_LidarObject3d[i].atYmax.y,self.ar_LidarObject3d[i].atYmax.z,1]])
            st_Lidar2Camera = np.matmul(self.FinalMatrix, np.transpose(st_LidarPosition3d))
            st_LidarObjectMax = [int(st_Lidar2Camera[0][0] / st_Lidar2Camera[2][0]), 0]

            # Min point 변환 (최소 y값 지점 변환)
            st_LidarPosition3d = np.array([[self.ar_LidarObject3d[i].atYmin.x,self.ar_LidarObject3d[i].atYmin.y,self.ar_LidarObject3d[i].atYmin.z,1]])
            st_Lidar2Camera = np.matmul(self.FinalMatrix, np.transpose(st_LidarPosition3d))
            st_LidarObjectMin = [int(st_Lidar2Camera[0][0] / st_Lidar2Camera[2][0]), 0]

            # zMax (최대 z값 변환)
            st_LidarPosition3d = np.array([[self.ar_LidarObject3d[i].atZmax.x,self.ar_LidarObject3d[i].atZmax.y,self.ar_LidarObject3d[i].atZmax.z,1]])
            st_Lidar2Camera = np.matmul(self.FinalMatrix, np.transpose(st_LidarPosition3d))
            st_LidarObjectMax[1] = int(st_Lidar2Camera[1][0] / st_Lidar2Camera[2][0])

            # zMin (최소 z값 변환)
            st_LidarPosition3d = np.array([[self.ar_LidarObject3d[i].xMin,self.ar_LidarObject3d[i].y,self.ar_LidarObject3d[i].zMin,1]])
            st_Lidar2Camera = np.matmul(self.FinalMatrix, np.transpose(st_LidarPosition3d))
            st_LidarObjectMin[1] = int(st_Lidar2Camera[1][0] / st_Lidar2Camera[2][0]) 

            # core point가 이미지 안에 존재할 경우 (바운딩 박스가 이미지 내 존재하는 경우 보정)
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

        self.arst_LidarBBoxes = LidarBBoxes #라이다 박스 저장

    def DrawBBox(self): #라이다 및 카메라 바운딩 박스를 그리는 함수
        # Camera BBox
        tmp_BBoxImage = self.st_BGRImg.copy()
        CameraBBoxes = []
        tmp_ObjCntCamera = self.u16_ObjCntCamera
        tmp_CamObject = self.arst_CamObject
        for i in range(tmp_ObjCntCamera): #감지된 카메라 객체 수 만큼 반복
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

    def FusionProcessing(self): #라이다와 카메라의 객체를 비교하여 IOU를 계산하는 함수
        # cal iou 
        tmp_BBoxImage = self.BBoxImage.copy()
        for CameraIndex, CameraBBox in enumerate(self.arst_CamBBoxes): 

            for LiDarIndex, LiDarBBox in enumerate(self.arst_LidarBBoxes):

                IoU = self.Iou(LiDarBBox, CameraBBox) #IOU 계산
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