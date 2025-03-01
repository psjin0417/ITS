#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage
from camera_ITS.msg import Yolo_object_arr, object, Coordinate
from ultralytics import YOLO
import math 

    
class st_LaneDetection:
    def __init__(self):
    
        # Subscriber
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback_Image)
        rospy.Subscriber("/Cam/right", CompressedImage, self.callback_right_Image)
        rospy.Subscriber("/Cam/left", CompressedImage, self.callback_left_Image)
        
        self.st_Publisher=rospy.Publisher("/YOLO/Object" , Yolo_object_arr, queue_size=5)

        self.st_BGRImg = None
        self.st_right_Img = None
        self.st_left_Img = None
        self.rate = rospy.Rate(50) 
        
        # model
        self.model = YOLO("yolo-Weights/yolo11m.pt")
        #self.model = YOLO("/home/seojin/AIM/best3.pt")
        self.classNames = ["person","","car"]

        self.speed_limit = 0

        self.SpeedSignDetection()

    def callback_Image(self,msg):
        np_arr=np.frombuffer(msg.data,np.uint8)
        img_bgr=cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
        if img_bgr is not None:
            self.st_BGRImg=img_bgr

    def callback_right_Image(self, msg):
        np_arr=np.frombuffer(msg.data,np.uint8)
        img_bgr=cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
        if img_bgr is not None:
            self.st_right_Img=img_bgr

    def callback_left_Image(self, msg):
        np_arr=np.frombuffer(msg.data,np.uint8)
        img_bgr=cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
        if img_bgr is not None:
            self.st_left_Img=img_bgr  
    
    def SpeedSignDetection(self):

        while self.st_BGRImg is None or self.st_right_Img is None or self.st_left_Img is None:
            pass

        while not rospy.is_shutdown():

            PublishMsg = Yolo_object_arr()

            # Change HSV Image
            front_img = self.st_BGRImg.copy()
            right_img = self.st_right_Img.copy()
            left_img = self.st_left_Img.copy()
            img = [front_img,left_img,right_img]

            # YOLO 모델을 사용하여 프레임 추론
            result_F = self.model(front_img, stream=True, verbose=False)
            result_L = self.model(left_img, stream=True, verbose=False)
            result_R = self.model(right_img, stream=True, verbose=False)
            arr_results = [result_F,result_L, result_R]

           
            # 추론 결과 가져오기
            count = 0
            object_arr = []
            for idx, results in enumerate(arr_results):
                objects = []
                for result in results:

                    
                    boxes = result.boxes  # 바운딩 박스 정보

                    for i,box in enumerate(boxes):
                        
                        confidence = math.ceil((box.conf[0]*100))/100 #0~1 사이값 소수점 아래 4자리 -> 2번쨰 자리 반올림
                        #if confidence > 0.3:
                        if int(box.cls[0] == 2) or int(box.cls[0] == 0) and confidence > 0.4:
                            count = count + 1

                            # bounding box
                            x1, y1, x2, y2 = box.xyxy[0] # 좌상단,우하단 
                            width = int(abs(x2-x1))
                            
                            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values (텐서형태로 반환하기 때문에)
                            
                            obj = object(CamNum = idx, w = width, BB =  Coordinate(x = int((x1+x2)/2) , y = y2 ))
                            
                            PublishMsg.data.append(obj)
                            # put box in cam
                            cv2.rectangle(img[idx], (x1, y1), (x2, y2), (255, 0, 255), 3)

                            # class name
                            #cls = int(box.cls[0])
               

            self.st_Publisher.publish(PublishMsg)
    
            # 디스플레이 (옵션)
            # cv2.imshow("YOLO 1", front_img)
            # cv2.imshow("YOLO 2", left_img)
            # cv2.imshow("YOLO 3", right_img)
            cv2.waitKey(1)
            self.rate.sleep()


    
if __name__ == '__main__':
    rospy.init_node('YoloDetection', anonymous=True)
    st_LaneDetection()
    rospy.spin()