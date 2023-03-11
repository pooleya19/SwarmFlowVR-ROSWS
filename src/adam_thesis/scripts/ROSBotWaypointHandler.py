#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion, Vector3
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import BatteryState
import math
from PID import PID

class ROSBotHandler:
	def __init__(self, name):
		self.name = name
		self.recentPose = None
		self.distancePID = None
		self.anglePID = None
		self.targetWaypoint = None
		self.targetWaypointTime = None

		self.battery = None

		# ===== Init parameters =====
		# Movement parameters
		self.turnSpeed = 2
		self.moveSpeed = 0.5
		self.maxAngleOffset = 0 #15 * math.pi/180 # radians
		self.maxLinearOffset = 0.2 #m?
		# Timeout
		self.waypointTimeout = 0.5 # [seconds], stop moving if no waypoints after this amount of time

		# Init PID controllers
		self.distancePID = PID(0.7,0,0)
		self.anglePID = PID(4,0,0.5)

		# Setup pose subscriber
		rospy.Subscriber(self.getTopicName("pose"), PoseStamped, self.callback_pose)
		# Setup /cmd_vel publisher
		self.pub_RosbotVel = rospy.Publisher(self.getTopicName("cmd_vel"), Twist, queue_size=1)
		# Setup /targetWaypoint subscriber
		rospy.Subscriber(self.getTopicName("targetWaypoint"), Vector3, self.callback_targetWaypoint)
		# Setup /battery subscriber
		rospy.Subscriber(self.getTopicName("battery"), BatteryState, self.callback_battery)

		# Send log message
		rospy.loginfo("Handler [%s] Started!", self.name)

	def callback_pose(self, data):
		self.recentPose = data
		# rospy.loginfo("Received pose data")

	def callback_targetWaypoint(self, data):
		self.targetWaypoint = data
		self.targetWaypointTime = rospy.get_time()

	def callback_battery(self, data):
		self.battery = data.voltage

	def waypointMove(self):
		# Check if no waypoint or pose data
		if self.targetWaypoint == None or self.recentPose == None:
			self.stopMoving()
			return
		# Check for timeout
		currentTime = rospy.get_time()
		elapsedTime = currentTime - self.targetWaypointTime
		if elapsedTime > self.waypointTimeout:
			self.stopMoving()
			return

		position = self.recentPose.pose.position
		orientation = self.recentPose.pose.orientation
		(roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
		yaw = (yaw+2*math.pi) % (2*math.pi)
		#rospy.loginfo("RPY: (%.3f, %.3f, %.3f)", roll, pitch, yaw)

		posOffset = Vector3(self.targetWaypoint.x - position.x, self.targetWaypoint.y - position.y, self.targetWaypoint.z - position.z)

		targetAngle = math.atan2(posOffset.y, posOffset.x)
		targetAngle = (targetAngle+2*math.pi) % (2*math.pi)

		# Deal with angle
		degToRad = math.pi / 180
		angleDiff = targetAngle - yaw
		angleDiff = ((angleDiff + math.pi) % (2*math.pi)) - math.pi
		angleDiffAbs = abs(angleDiff)
		anglePIDMag = self.anglePID.step(angleDiff,rospy.get_time())
		zOmega = self.deadzone(anglePIDMag,0.2*degToRad)

		# Deal with distance
		if angleDiffAbs < 50*degToRad:
			distance = math.sqrt(math.pow(posOffset.x,2) + math.pow(posOffset.y,2))
			distancePIDMag = self.distancePID.step(distance,rospy.get_time())
			xVel = self.deadzone(distancePIDMag,0.05)
			xVel = min(xVel, 0.7)
		else:
			xVel = 0

		# Create and publish twist
		newTwist = self.generateTwist(xVel,0,0,0,0,zOmega)
		self.pub_RosbotVel.publish(newTwist)
		rospy.loginfo("Handler [%s]:  ZOmega: %.2f [deg/s], XVel: %.2f", self.name, zOmega/degToRad, xVel)
		return

	def getVoltage(self):
		return self.battery

	def getPosition(self):
		if self.recentPose == None:
			return None
		else:
			return self.recentPose.pose.position

	def getTopicName(self, topicName):
		return "/" + self.name + "/" + topicName

	def stopMoving(self):
		twistZero = Twist(Vector3(0,0,0),Vector3(0,0,0))
		self.pub_RosbotVel.publish(twistZero)

	def deadzone(self, x, xmin):
		if x < xmin and x > -xmin:
			return 0
		return x

	def generateTwist(self, xVel, yVel, zVel, xOmega, YOmega, ZOmega):
		newLinear = Vector3(xVel, yVel, zVel)
		newAngular = Vector3(xOmega, YOmega, ZOmega)
		newTwist = Twist(newLinear, newAngular)
		return newTwist