#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
# from ultralytics.yolo.v8.detect.predict import DetectionPredictor
import cv2

def image_callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    name= ['']
    model = YOLO("/home/shadow1/face_detection_ws/src/face_detection/scripts/yolov8n.pt")
    results = model.predict(cv_image, show=False, conf=0.8)
    # print(results.__getitem__(0))
    for result in results:
        detection_count = result.boxes.shape[0]
        for i in range(detection_count):
            cls = int(result.boxes.cls[i].item())
            name.append(result.names[cls])
            confidence = float(result.boxes.conf[i].item())
            bounding_box = result.boxes.xyxy[i].cpu().numpy()
            x = int(bounding_box[0])
            y = int(bounding_box[1])
            width = int(bounding_box[2] - x)
            height = int(bounding_box[3] - y)
           # result.
    # result_msg = String()
    # result_msg.data = str(results)
    # result_pub.publish(result_msg)
    del name[0]
    print(name)
if __name__ == '__main__':

   
    rospy.init_node('yolo_detection')
    rate = rospy.Rate(15)
  
    result_pub = rospy.Publisher('yolo_result', String, queue_size=10)

   
    image_sub = rospy.Subscriber('/camera/image_raw', Image, image_callback)

    rospy.spin()