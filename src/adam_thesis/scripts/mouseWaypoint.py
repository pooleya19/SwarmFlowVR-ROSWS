#!/usr/bin/env python3

import rospy
import numpy
from geometry_msgs.msg import Vector3
from Xlib import display
from Xlib.ext import randr

def mouseWaypoint():
	# initialize node
	rospy.init_node('mouseWaypoint', anonymous = True)
	# Setup /targetWaypoint publisher
	pub_targetWaypoint = rospy.Publisher("targetWaypoint", Vector3, queue_size=1)
	rate = rospy.Rate(10) # 10hz
		
	# Get Display Dependent Parameters
	d = display.Display()
	screen = d.screen()
	window = screen.root.create_window(0, 0, 1, 1, 1, screen.root_depth)
	res = randr.get_screen_resources(window)

	# mouse to room settings
	# mouse +x -> room -y
	# mouse +y -> room +x
	roomCenter_x = 0.3
	roomCenter_y = 0
	roomSize_x = 2.8
	roomSize_y = 2.4
	
	while not rospy.is_shutdown():
		# Get display parameters
		resolution_x = res.modes[0].width	# i.e. 1920
		resolution_y = res.modes[0].height	# i.e. 1080
		resolution_bound = min(resolution_x, resolution_y)/2
		screenCenter_x = resolution_x / 2.0
		screenCenter_y = resolution_y / 2.0
		#rospy.loginfo("Detected Res: (%.2f, %.2f)", resolution_x, resolution_y) 
		
		# Get mouse position and scale by window size
		mouseData = display.Display().screen().root.query_pointer()._data
		mousePosOffset_x = mouseData["root_x"]-screenCenter_x
		mousePosOffset_y = -1*(mouseData["root_y"]-screenCenter_y)

		mousePosScaled_x = mousePosOffset_x / resolution_bound
		mousePosScaled_y = mousePosOffset_y / resolution_bound

		if mousePosScaled_x <= 1 and mousePosScaled_x >= -1 and mousePosScaled_y <= 1 and mousePosScaled_y >= -1:
			rospy.loginfo("MousePoseScaled: (%.2f, %.2f)",mousePosScaled_x,mousePosScaled_y)

			roomPos_x = roomCenter_x + roomSize_x/2 * mousePosScaled_y
			roomPos_y = roomCenter_y + roomSize_y/2 * -mousePosScaled_x 

			targetWaypoint = Vector3(roomPos_x, roomPos_y, 0)
			rospy.loginfo("TargetWaypoint: (%.2f, %.2f)", targetWaypoint.x, targetWaypoint.y)

			# Publish
			pub_targetWaypoint.publish(targetWaypoint)

		rate.sleep()


if __name__ == '__main__':
	mouseWaypoint()