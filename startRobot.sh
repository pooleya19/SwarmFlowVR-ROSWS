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
# roslaunch ~/husarion_ws/src/husarion_ros/launch/rosbot_drivers_pro.launch