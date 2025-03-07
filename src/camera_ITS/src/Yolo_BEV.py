#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage
from camera_ITS.msg import Yolo_object_arr, object, Coordinate
from geometry_msgs.msg import Polygon, Point
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
        self.rate = rospy.Rate(20) 
        
        # model
        #self.model = YOLO("yolo-Weights/yolo11n.pt")
        self.model = YOLO("yolo11s-seg.pt")
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

    # def image_process(self, cam_num,image,PublishMsg):
    #     results = self.model(image, stream=True, verbose=False)
    #     for result in results:

    #         boxes = result.boxes  # 바운딩 박스 정보

    #         for i,box in enumerate(boxes):
                
    #             confidence = math.ceil((box.conf[0]*100))/100 #0~1 사이값 소수점 아래 4자리 -> 2번쨰 자리 반올림
    #             #if confidence > 0.3:
    #             if int(box.cls[0] == 2) or int(box.cls[0] == 0) and confidence > 0.4:

    #                 # bounding box
    #                 x1, y1, x2, y2 = box.xyxy[0] # 좌상단,우하단 
    #                 width = int(abs(x2-x1))
                    
    #                 x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values (텐서형태로 반환하기 때문에)
                    
    #                 obj = object(CamNum = cam_num, w = width*3, BB =  Coordinate(x = int((x1+x2)/2)*3 , y = y2*3 ))
                    
    #                 PublishMsg.data.append(obj)
    #                 # put box in cam
    #                 cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 255), 3)

    #                 # class name
    #                 #cls = int(box.cls[0])
    def image_process(self, cam_num,image,PublishMsg):
        results = self.model(image, stream=True, verbose=False)
        for result in results:
                # 마스크 (세그멘테이션 결과)
                if result.masks is not None:
                    masks = result.masks.data.cpu().numpy()  # 마스크 데이터 (클래스별 이진 마스크)
                    boxes = result.boxes.xyxy.cpu().numpy()  # 바운딩 박스 좌표
                    classes = result.boxes.cls.cpu().numpy()  # 클래스 ID
                    confidences = result.boxes.conf.cpu().numpy()  # 신뢰도

                    # 원본 이미지에 결과 그리기
                    for i, mask in enumerate(masks):
                        if confidences[i] > 0.3 and classes[i] == 2:
                            print(classes[i])
                            # 마스크를 0-255 범위로 변환
                            mask = (mask * 255).astype(np.uint8)
                            mask = cv2.resize(mask, (image.shape[1], image.shape[0]))  # 이미지 크기에 맞게 조정

                            # 마스크 픽셀의 좌표 추출
                            y_coords, x_coords = np.where(mask > 0)  # 마스크가 있는 모든 픽셀의 (y, x) 좌표

                            if len(y_coords) > 0:  # 마스크 픽셀이 존재할 때만 처리
                                # 최하단 좌표 (y가 가장 큰 값)
                                bottom_idx = np.argmax(y_coords)
                                bottom_x, bottom_y = x_coords[bottom_idx], y_coords[bottom_idx]
                                cv2.circle(image,(bottom_x,bottom_y),3,(0,0,255),3)
                                

                            # # 마스크를 색상으로 표시 (예: 빨간색)
                            # color_mask = np.zeros_like(image)
                            # color_mask[mask > 0] = [0, 0, 255]  # BGR 형식 (빨간색)

                            # 원본 이미지와 마스크 합성
                            #front_img = cv2.addWeighted(front_img, 0.7, color_mask, 0.3, 0)

                            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                            if contours:
                                # 가장 큰 컨투어만 선택 (효율성을 위해)
                                contour = max(contours, key=cv2.contourArea)
                            
                                # 컨투어 점 단순화 (Douglas-Peucker 알고리즘, 점 개수 줄이기)
                                epsilon = 0.01 * cv2.arcLength(contour, True)
                                approx_contour = cv2.approxPolyDP(contour, epsilon, True)
                            x1, y1, x2, y2 = map(int, boxes[i])

                            polygon = Polygon()
                            for point in approx_contour:
                                x, y = point[0]
                                if y > y2:
                                    y = y2 
                                polygon.points.append(Point(x=float(x), y=float(y)))
                                
                            obj = object(CamNum = cam_num, w = 0, BB =  Coordinate(x = int(bottom_x)*2 , y = int(y2*2) ), contour = polygon)
                            PublishMsg.data.append(obj)
                            # 경계선 그리기 (노란색, 두께 2)
                            cv2.drawContours(image, contours, -1, (0, 255, 255), 2)


    
    def SpeedSignDetection(self):

        while self.st_BGRImg is None or self.st_right_Img is None or self.st_left_Img is None:
            pass

        while not rospy.is_shutdown():

            self.PublishMsg = Yolo_object_arr()

            # Change HSV Image
            front_img = self.st_BGRImg.copy()
            right_img = self.st_right_Img.copy()
            left_img = self.st_left_Img.copy()
            front_img = cv2.resize(front_img, (510, 360))
            right_img = cv2.resize(right_img, (510, 360))
            left_img = cv2.resize(left_img, (510, 360))
            images = [(0, front_img), (1, left_img), (2, right_img)]

            # with ThreadPoolExecutor(max_workers=3) as executor:
            #     executor.map(lambda args: self.image_process(args[0], args[1], self.PublishMsg), images)

            self.image_process(0, front_img, self.PublishMsg)
            self.image_process(1, left_img, self.PublishMsg)
            self.image_process(2, right_img, self.PublishMsg)

            
                    # 결과 이미지 표시
            cv2.imshow("YOLOv8 Segmentation", front_img)


            self.st_Publisher.publish(self.PublishMsg)
    
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