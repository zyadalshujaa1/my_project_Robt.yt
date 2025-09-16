cd /home/shadow1/face_detection_ws
source devel/setup.sh
export ROS_MASTER_URI=http://localhost:11311
#export ROS_IP=192.168.45.227
#export ROS_HOSTNAME=192.168.45.227

roslaunch face_detection cv.launch

