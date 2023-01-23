#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion, Vector3
from tf.transformations import euler_from_quaternion
import math
import numpy

lastReceievedMsg = 0
killDelay = 0.5 # send kill movement messages after 0.5 seconds of no new messages

def movementKiller():
	# initialize node
	rospy.init_node('movementKiller', anonymous = True)
	rospy.loginfo("MovementKiller Node Started!")
	# Setup /cmd_vel  subscriber
	rospy.Subscriber("/cmd_vel", Twist, callback_cmdvel)
	# Setup /cmd_vel publisher
	pub_RosbotVel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
	killTwist = Twist(Vector3(0,0,0),Vector3(0,0,0))

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		if rospy.get_time() - lastReceievedMsg >= killDelay:
			pub_RosbotVel.publish(killTwist)


def callback_cmdvel(data):
	global lastReceievedMsg
	lastReceievedMsg = rospy.get_time()


if __name__ == '__main__':
	movementKiller()