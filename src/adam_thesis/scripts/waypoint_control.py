#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion, Vector3
from tf.transformations import euler_from_quaternion
import math
import numpy
from PID import PID

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
distancePID = None
anglePID = None
targetWaypoint = None

degToRad = math.pi / 180

def callback_Rosbot01Pose(data):
	global recentPose
	recentPose = data

def callback_targetWaypoint(data):
	global targetWaypoint
	targetWaypoint = data

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
	global distancePID, anglePID

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

	# Deal with angle
	angleDiff = targetAngle - yaw
	angleDiff = ((angleDiff + math.pi) % (2*math.pi)) - math.pi
	angleDiffAbs = abs(angleDiff)
	anglePIDMag = anglePID.step(angleDiff,rospy.get_time())
	zOmega = deadzone(anglePIDMag,0.2*degToRad)
	
	# Deal with distance
	if angleDiffAbs < 50*degToRad:
		distance = math.sqrt(math.pow(posOffset.x,2) + math.pow(posOffset.y,2))
		distancePIDMag = distancePID.step(distance,rospy.get_time())
		xVel = deadzone(distancePIDMag,0.05)
		xVel = min(xVel, 0.7)
	else:
		xVel = 0

	# Create and publish twist
	newTwist = generateTwist(xVel,0,0,0,0,zOmega)
	pub_RosbotVel.publish(newTwist)
	rospy.loginfo("ZOmega: %.2f [deg/s], XVel: %.2f",zOmega/degToRad,xVel)
	return


def waypointControl():
	global distancePID, anglePID, targetWaypoint

	# configure parameters
	listenForWaypoints = True
	customWaypointDelay = 10 #seconds


	# initialize node
	rospy.init_node('waypointControl', anonymous = True)
	rospy.loginfo("WaypointControl Node Started!")
	# Setup /Rosbot01/pose subscriber
	rospy.Subscriber("/Rosbot01/pose", PoseStamped, callback_Rosbot01Pose)
	# Setup /cmd_vel publisher
	pub_RosbotVel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
	# Init PID controllers
	distancePID = PID(0.7,0,0)
	anglePID = PID(4,0,0.5)
	# Setup /targetWaypoint subscriber
	if listenForWaypoints:
		rospy.Subscriber("/targetWaypoint", Vector3, callback_targetWaypoint)

	# Wait for pose data before continuing
	waitForPoseData()

	startTime = rospy.get_time()

	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		if not listenForWaypoints:
			currentTime = rospy.get_time()
			elapsedTime = currentTime-startTime
			modTime = elapsedTime % (4*customWaypointDelay)

			if modTime < 1*customWaypointDelay:
				targetWaypoint = Vector3(0.3+1.4, 0-1.2, 0)
			elif modTime < 2*customWaypointDelay:
				targetWaypoint = Vector3(0.3+1.4, 0+1.2, 0)
			elif modTime < 3*customWaypointDelay:
				targetWaypoint = Vector3(0.3-1.4, 0+1.2, 0)
			elif modTime < 4*customWaypointDelay:
				targetWaypoint = Vector3(0.3-1.4, 0-1.2, 0)

		if targetWaypoint != None:
			waypointMove(pub_RosbotVel, targetWaypoint)
		else:
			rospy.loginfo("Waypoint is none.")

		rate.sleep()


def generateTwist(xVel, yVel, zVel, xOmega, YOmega, ZOmega):
	newLinear = Vector3(xVel, yVel, zVel)
	newAngular = Vector3(xOmega, YOmega, ZOmega)
	newTwist = Twist(newLinear, newAngular)
	return newTwist

def remap(x, xmin, xmax, ymin, ymax):
	if(x <= xmin):
		return ymin
	if(x >= xmax):
		return ymax
	return (x-xmin)/(xmax-xmin)*(ymax-ymin) + ymin

def deadzone(x, xmin):
	if x < xmin and x > -xmin:
		return 0
	return x 

def publishRosBot(xVel, zOmega):
	goodXVel = 1
	goodZOmegaVel = 2
	pub_RosbotVel.publish(msg)


if __name__ == '__main__':
	waypointControl()