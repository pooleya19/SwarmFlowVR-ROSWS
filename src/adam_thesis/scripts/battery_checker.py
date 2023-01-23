#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import BatteryState

batteryState = None

def batteryChecker():
	# initialize node
	rospy.init_node('batteryChecker', anonymous = True)
	rospy.loginfo("BatteryChecker Node Started!")
	# Setup /battery  subscriber
	rospy.Subscriber("/battery", BatteryState, callback_battery)

	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		if batteryState != None:
			rospy.loginfo("Battery Voltage: %.2f", batteryState.voltage)
		rate.sleep()


def callback_battery(data):
	global batteryState
	batteryState = data


if __name__ == '__main__':
	batteryChecker()