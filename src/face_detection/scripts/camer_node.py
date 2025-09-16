#!/usr/bin/python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class VideoPublisher:
    def __init__(self):
        self.image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=1)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0) # Change this to the index of your camera if needed
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640) # Set the camera resolution to 640x480
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.rate = rospy.Rate(30) # Set the publishing rate to 30 Hz

    def publish_video(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()

            if ret:
                try:
                    # Display the image using OpenCV
                    #cv2.imshow("Video window", frame)
                    #cv2.waitKey(1)

                    # Publish the image as a ROS message
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
                except CvBridgeError as e:
                    rospy.logerr(e)

            self.rate.sleep()

        # Release the camera and close the OpenCV window when the node is shut down
        self.cap.release()
       # cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('video_publisher')
    vp = VideoPublisher()
    vp.publish_video()











    
