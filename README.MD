ROS package to FOXBOT.

Credits to: Douglas Bertol, Maurício Ferreira and André Leal from UDESC — CCT.

AGV_PKG -> Automated Guided Vehicle Package

Commands:

export ROS_MASTER_IP=http://192.168.1.16/11311/
export ROS_IP=192.168.1.16 
source devel/setup.zsh 

rosrun agv_pkg calibration.py
rosbag record [topic]
