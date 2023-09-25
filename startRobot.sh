#!/usr/bin/env bash
if [ "$1" != "" ]; then
	robot=$1

	# Check if device is online
	timeout=2
	ping $robot -c 1 -W $timeout > /dev/null
	if [ $? == 0 ]; then
		ssh -t husarion@$robot '~/scripts/startRosbot.sh'
	else
		echo "$robot is offline."
	fi
else
	echo "Usage: startRobot.sh [ROSbot alias]"
fi
# Each startRosbot.sh script on the ROSbots should perform same functionality as last few lines of ~/.bashrc
# i.e. define ROS_IP and ROS_MASTER and source the ROS distribution and the ROS workspace

# roslaunch ~/husarion_ws/src/husarion_ros/launch/rosbot_drivers_pro.launch