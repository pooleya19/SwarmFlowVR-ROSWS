#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion, Vector3
from tf.transformations import euler_from_quaternion
import math
import numpy

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


# init variable to store /Rosbot01/pose
recentPose = None

def callback_Rosbot01Pose(data):
	global recentPose
	recentPose = data


def waitForPoseData():
	# Wait for pose data before continuing
	fastRate = rospy.Rate(50)
	while not rospy.is_shutdown():
		if recentPose == None:
			rospy.loginfo("No pose data!")
		else:
			rospy.loginfo("Pose data received!")
			break		
		fastRate.sleep()


def waypointMove(pub_RosbotVel, targetWaypoint):
	turnSpeed = 2
	moveSpeed = 0.5
	maxAngleOffset = 0 #15 * math.pi/180 # radians
	maxLinearOffset = 0.2 #m?

	twistZero = Twist(Vector3(0,0,0),Vector3(0,0,0))
	if targetWaypoint == None:
		pub_RosbotVel.publish(twistZero)
		return

	position = recentPose.pose.position
	orientation = recentPose.pose.orientation
	(roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
	yaw = (yaw+2*math.pi) % (2*math.pi)
	#rospy.loginfo("RPY: (%.3f, %.3f, %.3f)", roll, pitch, yaw)

	posOffset = Vector3(targetWaypoint.x - position.x, targetWaypoint.y - position.y, targetWaypoint.z - position.z)

	targetAngle = math.atan2(posOffset.y, posOffset.x)
	targetAngle = (targetAngle+2*math.pi) % (2*math.pi)

	angleDiff = targetAngle - yaw
	angleDiff = ((angleDiff + math.pi) % (2*math.pi)) - math.pi

	# Deal with angle first
	degToRad = math.pi / 180
	angleDiffAbs = abs(angleDiff)

	if angleDiffAbs > 45 * degToRad:
		zOmegaMag = 90 * degToRad
	elif angleDiffAbs > 20 * degToRad:
		zOmegaMag = 60 * degToRad
	elif angleDiffAbs > 5 * degToRad:
		zOmegaMag = 25 * degToRad
	else:
		zOmegaMag = 5 * degToRad

	zOmega = numpy.sign(angleDiff) * zOmegaMag

	distance = math.sqrt(math.pow(posOffset.x,2) + math.pow(posOffset.y,2))

	if angleDiffAbs > 30*degToRad or distance < 0.1:
		xVel = 0
	else:
		xVel = (1-angleDiffAbs/90) * distance * 0.5
		#xVel = min(xVel,0.5)

	newTwist = generateTwist(xVel,0,0,0,0,zOmega)
	pub_RosbotVel.publish(newTwist)

	#rospy.loginfo("AngleDiff [deg]: %.2f, zOmega [deg]: %.2f", angleDiff/degToRad, zOmega/degToRad)
	rospy.loginfo("AngleDiffAbs [deg]: %.2f, Distance: %.2f, xVel: %.8f", angleDiffAbs/degToRad, distance, xVel)
	return


def waypointControl():
	# initialize node
	rospy.init_node('waypointControl', anonymous = True)
	rospy.loginfo("WaypointControl Node Started!")
	# Setup /Rosbot01/pose subscriber
	rospy.Subscriber("/Rosbot01/pose", PoseStamped, callback_Rosbot01Pose)
	# Setup /cmd_vel publisher
	pub_RosbotVel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

	# Wait for pose data before continuing
	waitForPoseData()

	startTime = rospy.get_time()
	targetWaypoint = None
	waypointDelay = 10 # seconds

	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		currentTime = rospy.get_time()
		elapsedTime = currentTime-startTime
		modTime = elapsedTime % (4*waypointDelay)

		if modTime < 1*waypointDelay:
			targetWaypoint = Vector3(0.3+2.1, 0-1.8, 0)
		elif modTime < 2*waypointDelay:
			targetWaypoint = Vector3(0.3+2.1, 0+1.8, 0)
		elif modTime < 3*waypointDelay:
			targetWaypoint = Vector3(0.3-2.1, 0+1.8, 0)
		elif modTime < 4*waypointDelay:
			targetWaypoint = Vector3(0.3-2.1, 0-1.8, 0)

		waypointMove(pub_RosbotVel, targetWaypoint)

		rate.sleep()


def generateTwist(xVel, yVel, zVel, xOmega, YOmega, ZOmega):
	newLinear = Vector3(xVel, yVel, zVel)
	newAngular = Vector3(xOmega, YOmega, ZOmega)
	newTwist = Twist(newLinear, newAngular)
	return newTwist


if __name__ == '__main__':
	waypointControl()