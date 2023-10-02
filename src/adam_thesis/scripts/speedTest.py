#!/usr/bin/env python3

import rospy
import numpy as np
from matplotlib import pyplot as plt
from geometry_msgs.msg import Vector3, PoseStamped
from tf.transformations import euler_from_quaternion
import math
import os

rosbotPoseSeq = 0
rosbotPose = None

def callback_pose(data):
	global rosbotPoseSeq
	global rosbotPose
	header = data.header
	seq = header.seq
	if seq > rosbotPoseSeq:
		rosbotPoseSeq = seq
		rosbotPose = data


def speedTest():
	targetWaypointA = Vector3(-2.4, 0, 0)
	targetWaypointB = Vector3(1.8, 0, 0)
	currentWaypointNum = 0

	commandPeriod = 8 # [s]
	timeLastCommand = -10000
	commandNum = 0
	maxCommandNum = 4

	fileName = "positionData.npy"
	figFileName = "positionDataFig.png"


	# Initialize node
	rospy.init_node('speedTest', anonymous = True)

	rosRefreshRate = 10 # [Hz]

	# Define rosbot name
	rosbotName = "rosbot01"

	# Init publisher
	topicNamePub = "/" + rosbotName + "/targetWaypoint"
	pub_targetWaypoint = rospy.Publisher(topicNamePub, Vector3, queue_size=1)

	# Init subscriber
	topicNameSub = "/" + rosbotName + "/pose"
	rospy.Subscriber(topicNameSub, PoseStamped, callback_pose)

	rate = rospy.Rate(rosRefreshRate)

	positions = None
	
	while not rospy.is_shutdown():

		currentTime = rospy.get_time()

		if (currentTime - timeLastCommand) >= commandPeriod:
			timeLastCommand = currentTime
			targetWaypoint = None
			if currentWaypointNum == 0:
				currentWaypointNum = 1
				targetWaypoint = targetWaypointB
			elif currentWaypointNum == 1:
				currentWaypointNum = 0
				targetWaypoint = targetWaypointA
			if commandNum < maxCommandNum:
				commandNum += 1
				print("Command ",commandNum,"/",maxCommandNum," given.",sep='')
			else:
				print("All done, shutting down.")
				rospy.signal_shutdown("Time to go.")

		pub_targetWaypoint.publish(targetWaypoint)

		if rosbotPose is not None:
			pos = rosbotPose.pose.position
			orientation = rosbotPose.pose.orientation
			(roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
			currentPosition = np.atleast_2d([currentTime, pos.x, pos.y, pos.z, yaw])
			if positions is None:
				positions = currentPosition
			else:
				positions = np.concatenate([positions, currentPosition], axis=0)
			print(positions.shape)

		rate.sleep()

	np.save(fileName, positions)
	rospy.loginfo("Saved %s", fileName)

	plt.figure()
	axisTitles = ["X","Y","Z","Yaw"]
	for i in range(4):
		plt.subplot(1,4,i+1)
		plt.plot(positions[:,0],positions[:,i+1])
		plt.title(axisTitles[i])
	plt.savefig(figFileName)

if __name__ == '__main__':
	speedTest()