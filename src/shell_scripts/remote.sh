cd /home/shadow1/zetaxbot
source devel/setup.sh
export ROS_MASTER_URI=http://ros.local:11311
export ROS_IP=192.168.133.227
export ROS_HOSTNAME=192.168.133.227

roslaunch zetaxbot_remote remote_interface.launch

