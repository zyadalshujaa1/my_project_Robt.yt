#!/usr/bin/env python3

import time
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import face_recognition
import os
import numpy as np
import time

rospy.init_node('face_recognition_node')

bridge = CvBridge()

path = '/home/shadow1/face_detection_ws/src/face_detection/scripts/persons'
images = []
classNames = []
personsList = os.listdir(path)




global frame1
for cl in personsList:
    curPersonn = cv2.imread(f'{path}/{cl}')
    images.append(curPersonn)
    classNames.append(os.path.splitext(cl)[0])
print(classNames)


def findEncodeings(image):
    encodeList = []
    for img in images:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        encode = face_recognition.face_encodings(img)
        for encoding in encode:
            encodeList.append(encoding)
    return encodeList

encodeListKnown = findEncodeings(images)
print('Encoding Complete.')

# ROS Image Publisher
image_pub = rospy.Publisher('face_recognition/image', Image, queue_size=10)

# ROS Name Publisher
name_pub = rospy.Publisher('face_recognition/name', String, queue_size=10)

def image_callback(ros_image):
    global frame1
    # Convert ROS Image message to OpenCV image
    frame = bridge.imgmsg_to_cv2(ros_image, desired_encoding="bgr8")
    frame1 =frame
    face_locations = face_recognition.face_locations(frame1)
    face_encodings = face_recognition.face_encodings(frame1, face_locations)

    for encodeface, faceLoc in zip(face_encodings, face_locations):
        matches = []
        faceDis = []

        for known_encoding in encodeListKnown:
            match = face_recognition.compare_faces([known_encoding], encodeface)
            matches.append(match[0])
            distance = face_recognition.face_distance([known_encoding], encodeface)
            faceDis.append(distance[0])

        matchIndex = np.argmin(faceDis)

        if matches[matchIndex]:
            name = classNames[matchIndex].upper()
            print(name)

            y1, x2, y2, x1 = faceLoc
            y1, x2, y2, x1 = y1*4, x2*4, y2*4, x1*4
            cv2.rectangle(frame1, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.rectangle(frame1, (x1, y2-35), (x2, y2), (0, 0, 255), cv2.FILLED)
            cv2.putText(frame1, name, (x1+6, y2-6), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 2)

            # Convert the frame to ROS Image message
            ros_image = bridge.cv2_to_imgmsg(frame1, encoding="bgr8")
            a=rospy.get_param('/face_recognition_publisher',False)
            if(a== True):
                image_pub.publish(ros_image)

            # Publish the name
            name_pub.publish(name)

    cv2.imshow('Face Recognition', frame1)
    cv2.waitKey(1)

def name_callback(data):
    global path
    global frame1
    # Name callback
    # frame = bridge.imgmsg_to_cv2(ros_image, desired_encoding="bgr8")
    name = data.data
    for i in range(3):
        
        
          
        
        frame_path = os.path.join(path, name + f'_{i}.jpg')
        cv2.imwrite(frame_path, frame1)
        time.sleep(2)
        print("Received name:", name)

    
    

# ROS Image Subscriber
image_sub = rospy.Subscriber('/camera/image_raw', Image, image_callback)

# ROS Name Subscriber
name_sub = rospy.Subscriber('/face_recognition/name_input', String, name_callback)

rospy.spin()