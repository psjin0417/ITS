#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage
from camera_ITS.msg import Yolo_bbox_arr, Yolo_bbox
from ultralytics import YOLO
from cv_bridge import CvBridgeError
import math 

    
class st_LaneDetection:
    def __init__(self):
    
        # Subscriber
        self.st_Subscriber=rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback_Image)
        self.st_Publisher=rospy.Publisher("/YOLO/BBox" , Yolo_bbox_arr, queue_size=5)

        self.st_BGRImg = None
        self.rate = rospy.Rate(50) 
        
        # model
        self.model = YOLO("yolo-Weights/yolo11m.pt")
        #self.model = YOLO("/home/seojin/AIM/best3.pt")
        self.classNames = ["person","","car"]

        self.speed_limit = 0

        self.SpeedSignDetection()

    def callback_Image(self,msg):
        try:
            np_arr=np.frombuffer(msg.data,np.uint8)
            img_bgr=cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
            if img_bgr is not None:
                self.st_BGRImg=img_bgr
        except CvBridgeError as e:
            print(e)
    
    def SpeedSignDetection(self):

        while self.st_BGRImg is None:
            pass

        while not rospy.is_shutdown():
            # Change HSV Image
            img = self.st_BGRImg.copy()


            # YOLO 모델을 사용하여 프레임 추론
            results = self.model(img, stream=True, verbose=False)

            PublishMsg = Yolo_bbox_arr()
            # 추론 결과 가져오기
            count = 0
            for result in results:

                
                boxes = result.boxes  # 바운딩 박스 정보
                max_confidence = 0
                max_idx = None
                for idx,box in enumerate(boxes):
                    
                    confidence = math.ceil((box.conf[0]*100))/100 #0~1 사이값 소수점 아래 4자리 -> 2번쨰 자리 반올림
                    #if confidence > 0.3:
                    if int(box.cls[0] == 2) or int(box.cls[0] == 0) and confidence > 0.3:
                        count = count + 1

                        # bounding box
                        x1, y1, x2, y2 = box.xyxy[0] # 좌상단,우하단 
                        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values (텐서형태로 반환하기 때문에)
                        
                        bbox_arr = Yolo_bbox(xMin = x1, yMin = y1, xMax = x2, yMax = y2)
                        
                        PublishMsg.yolo_bbox_arr.append(bbox_arr)
                        
                        # put box in cam
                        cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

                        # class name
                        cls = int(box.cls[0])



                        #print("Conf: ", confidence, ", Cls: ", classNames[cls])
            PublishMsg.objc = count
            self.st_Publisher.publish(PublishMsg)


                    
            # 디스플레이 (옵션)
            cv2.imshow("YOLO Detection", img)
            cv2.waitKey(1)
            self.rate.sleep()


    
if __name__ == '__main__':
    rospy.init_node('YoloDetection', anonymous=True)
    st_LaneDetection()
    rospy.spin()