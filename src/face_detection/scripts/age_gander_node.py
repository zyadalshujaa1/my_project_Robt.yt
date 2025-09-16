#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String

def faceBox(faceNet, frame):
    frameHeight=frame.shape[0]
    frameWidth=frame.shape[1]
    blob=cv2.dnn.blobFromImage(frame, 1.0, (300,300), [104,117,123], swapRB=False)
    faceNet.setInput(blob)
    detection=faceNet.forward()
    bboxs=[]
    for i in range(detection.shape[2]):
        confidence=detection[0,0,i,2]
        if confidence>0.7:
            x1=int(detection[0,0,i,3]*frameWidth)
            y1=int(detection[0,0,i,4]*frameHeight)
            x2=int(detection[0,0,i,5]*frameWidth)
            y2=int(detection[0,0,i,6]*frameHeight)
            bboxs.append([x1,y1,x2,y2])
            cv2.rectangle(frame, (x1,y1),(x2,y2),(0,255,0), 1)
    return frame, bboxs
rospy.init_node('age_gender_node')

# Initialize the CvBridge
bridge = CvBridge()

image_pub = rospy.Publisher('face_detection/image', Image, queue_size=10)
age_pub = rospy.Publisher('detection/age', String, queue_size=10)
gender_pub = rospy.Publisher('detection/gender', String, queue_size=10)


faceProto = "/home/shadow1/face_detection_ws/src/face_detection/scripts/opencv_face_detector.pbtxt"
faceModel = "/home/shadow1/face_detection_ws/src/face_detection/scripts/opencv_face_detector_uint8.pb"
faceNet = cv2.dnn.readNet(faceModel, faceProto)


ageProto = "/home/shadow1/face_detection_ws/src/face_detection/scripts/age_deploy.prototxt"
ageModel = "/home/shadow1/face_detection_ws/src/face_detection/scripts/age_net.caffemodel"
ageNet = cv2.dnn.readNet(ageModel, ageProto)

genderProto = "/home/shadow1/face_detection_ws/src/face_detection/scripts/gender_deploy.prototxt"
genderModel = "/home/shadow1/face_detection_ws/src/face_detection/scripts/gender_net.caffemodel"
genderNet = cv2.dnn.readNet(genderModel, genderProto)


MODEL_MEAN_VALUES = (78.4263377603, 87.7689143744, 114.895847746)
ageList = ['(0-2)', '(4-6)', '(8-12)', '(15-20)', '(20-25)', '(25-32)', '(38-43)', '(48-53)', '(60-100)']
genderList = ['Male', 'Female']


#video = cv2.VideoCapture('http://192.168.1.100:4747/video')

rate = rospy.Rate(20) 
padding = 20

def image_callback(msg):
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    if frame is not None:
        umat_image = cv2.UMat(frame)
        resized_image = cv2.resize(umat_image, (227, 227))
    
    frame, bboxs = faceBox(faceNet, frame)
    for bbox in bboxs:
        # face = frame[bbox[1]:bbox[3], bbox[0]:bbox[2]]
        face = frame[max(0, bbox[1] - padding):min(bbox[3] + padding, frame.shape[0] - 1),max(0, bbox[0] - padding):min(bbox[2] + padding, frame.shape[1] - 1)]
        blob=cv2.dnn.blobFromImage(face, 1.0, (227,227), MODEL_MEAN_VALUES, swapRB=False)
        genderNet.setInput(blob)
        genderPred=genderNet.forward()
        gender=genderList[genderPred[0].argmax()]
        
        ageNet.setInput(blob)
        agePred=ageNet.forward()
        age=ageList[agePred[0].argmax()]

        
        label="{},{}".format(gender,age)
        cv2.rectangle(frame,(bbox[0], bbox[1]-30), (bbox[2], bbox[1]), (0,255,0),-1) 
        cv2.putText(frame, label, (bbox[0], bbox[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2,cv2.LINE_AA)
      
        
       
        image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        a=rospy.get_param('/age_gender_publisher',False)
        if(a== True):
            image_pub.publish(image_msg)
        
        age_msg = String()
        age_msg.data = age
        
        gender_msg = String()
        gender_msg.data = gender
        age_pub.publish(age_msg)
        gender_pub.publish(gender_msg)
        
        # Save age and gender as ROS parameters
        rospy.set_param('/face_detection/age', age)
        rospy.set_param('/face_detection/gender', gender)
    
    cv2.imshow("Age-Gender", frame)
    cv2.waitKey(1)
if __name__ == '__main__':
    rospy.Subscriber('/camera/image_raw', Image, image_callback) 

    while not rospy.is_shutdown():
        rospy.spin()
        # rate.sleep()
        cv2.destroyAllWindows()