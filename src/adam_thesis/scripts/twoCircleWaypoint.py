#!/usr/bin/env python3

import rospy
import numpy
from geometry_msgs.msg import Vector3
from Xlib import display
from Xlib.ext import randr
import math

def twoCircleWaypoint():
	# Define parameters
	roomCenter_x = 0.3
	roomCenter_y = 0
	roomSize_x = 2.8
	roomSize_y = 2.4

	period = 15 # seconds

	rosRefreshRate = 10 #Hz

	# Define rosbot names
	rosbotNames = ["rosbot01", "rosbot02"]


	# Initialize node
	rospy.init_node('twoCircleWaypoint', anonymous = True)

	# Initialize publishers
	publishers = []
	for rosbotName in rosbotNames:
		topicName = "/" + rosbotName + "/targetWaypoint"
		newPublisher = rospy.Publisher(topicName, Vector3, queue_size=1)
		publishers.append(newPublisher)
	numBots = len(publishers)

	rate = rospy.Rate(rosRefreshRate)
	
	while not rospy.is_shutdown():

		currentTime = rospy.get_time()

		for i in range(0,numBots):
			publisher = publishers[i]
			phaseOffset = 2*math.pi / len(publishers) * i
			omega = 2*math.pi/period

			targetX = roomCenter_x + roomSize_x/2 * math.cos(omega * currentTime + phaseOffset)
			targetY = roomCenter_y + roomSize_y/2 * math.sin(omega * currentTime + phaseOffset)

			targetWaypoint = Vector3(targetX, targetY, 0)
			publisher.publish(targetWaypoint)
			rospy.loginfo("Published waypoint?")


		rate.sleep()


if __name__ == '__main__':
	twoCircleWaypoint()