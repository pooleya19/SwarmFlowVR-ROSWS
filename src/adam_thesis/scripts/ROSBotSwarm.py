#!/usr/bin/env python3

import rospy
from ROSBotWaypointHandler import ROSBotHandler
import math

"""
Quad1 Corner: 2.440, -1.800, 0.098
Quad2 Corner: 2.406, 1.809, 0.132
Quad3 Corner: -1.825, 1.820, 0.252
Quad4 Corner: -1.823, -1.812, 0.224

Quad4 plus 1 diag: -1.224, -1.203, 0.213
Quad4 plus 2 diag: -0.628, -0.595, 0.198

Each tile is 0.6 units long

x-axis increases towards door
y-axis increases towards windows

Center of room: 0.3, 0, 0.2
Size of room: 4.2, 3.6, 0

"""

def calcDistance(vecA, vecB):
	return math.sqrt(math.pow(vecA.x-vecB.x,2)+math.pow(vecA.y-vecB.y,2)+math.pow(vecA.z-vecB.z,2))

def main():
	#Define parameters
	closeThreshold = 0.5

	# Initialize node
	rospy.init_node('ROSBotSwarm', anonymous = True)
	rospy.loginfo("ROSBotSwarm Node Started!")

	# Init ROSBots
	botHandler1 = ROSBotHandler("rosbot01")
	botHandler2 = ROSBotHandler("rosbot02")
	botHandlers = [botHandler1, botHandler2]

	startTime = rospy.get_time()

	tooCloseCount = 0

	rate = rospy.Rate(20)
	manualShutdown = False
	while not rospy.is_shutdown() and not manualShutdown:
		batteryMsg = ""
		currentPositions = []
		tooClose = False
		for botHandler in botHandlers:
			# compare to current positions
			currentPosition = botHandler.getPosition()
			if currentPosition != None:
				for otherPosition in currentPositions:
					distance = calcDistance(currentPosition, otherPosition)
					#rospy.loginfo("NumPositions = %.2f, Dist = %.2f",len(currentPositions), distance)
					#rospy.loginfo("Dist: %.3f", distance)
					if distance < closeThreshold:
						rospy.logerr("ROSBOTS TOO CLOSE.")
						tooClose = True
						
				currentPositions.append(currentPosition)

			botHandler.waypointMove()

			battery = botHandler.getVoltage()
			if type(battery) == float:
				battery = round(battery,2)
			batteryMsg += "[" + botHandler.name + "] " + str(battery) + " V,  "
		rospy.loginfo(batteryMsg)

		if tooClose:
			tooCloseCount += 1
		else:
			tooCloseCount = 0

		if tooCloseCount > 5:
			manualShutdown = True

		rate.sleep()

	# Stop robots
	for botHandler in botHandlers:
		botHandler.stopMoving()
	rate.sleep()
	return

if __name__ == '__main__':
	main()