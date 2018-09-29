#!/usr/bin/env python
import cv2
import numpy as np
import math
import rospy
from std_msgs.msg import Int16MultiArray
import time
import tf 

pub=rospy.Publisher('array_values',Int16MultiArray,queue_size = 10)
rospy.init_node('pub_marray')
listener = tf.TransformListener()

#path = [[1,1],[1,2],[1,3],[2,3],[3,3],[3,2],[3,1],[2,1],[1,1]] ## Initialze the path to be followed to the destination 
#path = [[1,1],[1,2],[2,2],[2,3],[3,3],[3,4]]
#path = [[1,1],[2,2],[3,2],[4,1],[4,2],[4,3],[5,3],[6,3],[6,2]]
#path = [[1,1],[1,2],[1,3],[2,3],[3,3]]
#path = [[1,1],[1,2],[1,3],[2,3]]
#path = [[1,1],[1,2],[2,3]]
#path = [[1,1],[1,2],[2,2],[3,2],[3,1]]
#path = [(13, 26), (13, 25), (13, 24), (14, 23), (15, 22), (16, 21), (17, 20), (18, 19), (19, 19), (20, 19), (21, 19)]
path = [[1,1],[1,2],[1,3],[1,4],[2,4],[3,4],[4,4]]

finish = 0


#theta_0 = math.degrees(path[0][2])  ##Initial Orientation of the bot from the source
distance_per_pixel = 40
theta_0 = 0
delta = 0
rot = 0
yaw = 0
x = 3


def ack_listener():
    print "waiting for ack"
    return rospy.wait_for_message('/acknowledge', Int16MultiArray)

def ack_callback(data):
    global arduino_ack
    arduino_ack = data
    rospy.loginfo("received")
    return

rospy.Subscriber("/acknowledge", Int16MultiArray, ack_callback)


def move(angle,direction):
	pub_data = [2,0,angle,direction,0,0]
	if direction == -1:
		print 'Moving Left'
	elif direction == 1:
		print 'Moving Right'
	print 'Publishing Data ',pub_data
	time.sleep(1)
	
	m=Int16MultiArray(data=pub_data)
        rospy.loginfo(pub_data)
	pub.publish(m)
        ack_listener()
	

def find_yaw():
	flag = 0
	while flag == 0:
		try:
			(trans,rot) = listener.lookupTransform('/odom', '/kinect2_base_link', rospy.Time(0))
			flag = 1
		except:
			continue
	roll,pitch,yaw = tf.transformations.euler_from_quaternion(rot)
	return math.degrees(yaw)


def ang_diff(yaw,iyaw):
	print 'current yaw ',yaw,' initial yaw ',iyaw
	if yaw-iyaw < 180 and yaw-iyaw > -180:
		return abs(yaw-iyaw)
	else:
		return 360 - (abs(yaw-iyaw))

def find_distance():
	flag = 0
	while flag == 0:
		try:
			(trans,rot) = listener.lookupTransform('/odom', '/kinect2_base_link', rospy.Time(0))
			flag = 1
		except:
			continue
	return trans

def dis_diff(distance,idistance):
	diff = math.sqrt((distance[0]-idistance[0])**2 + (distance[1]-idistance[1])**2 + (distance[2]-idistance[2])**2)
	return diff

def dis_move(pub_data):
	m=Int16MultiArray(data=pub_data)
	rospy.loginfo(pub_data)
	pub.publish(m)
	ack_listener()
		

for i in range(0,len(path)-1):
	source = path[i]
	destination = path[i+1]
	print "source point is", source
	print "destination is", destination
	if destination[0]-source[0] == -1 and destination[1]-source[1] == 1:
		print "case 1"		
		theta_1 = 45
	elif((destination[0]-source[0] == 1) and (destination[1]-source[1] == 0)):
		print "case 2"
		theta_1 = 270
	elif((destination[1]-source[1] == 1) and (destination[0]-source[0] == 0)):
		print "case 3"
		theta_1 = 0
	elif((destination[0]-source[0] == -1) and (destination[1]-source[1] == 0)):
		print "case 4"
		theta_1 = 90
	elif((destination[0]-source[0] == -1) and (destination[1]-source[1] == -1)):
		print 'case 5'
		theta_1 = 135
	elif((destination[1]-source[1] == -1) and (destination[0]-source[0] == 0)):
		print "case 6"
		theta_1 = 180
	elif((destination[0]-source[0] == 1) and (destination[1]-source[1] == -1)):
		print "case 7"
		theta_1 = 225
	elif((destination[0]-source[0] == 1) and (destination[1]-source[1] == 1)):
		print "case 8"
		theta_1 = 315

	print "original theta 0", theta_0
	print "original theta 1", theta_1
	if theta_0 > 180:		
		theta_0 = -(360-theta_0)
	if theta_1 > 180:
		theta_1 = -(360-theta_1)
	print "altered theta 0", theta_0
	print "altered theta 1", theta_1
	time.sleep(1)

	delta = theta_1 - theta_0 		
	print "delta is", delta	
	if ((theta_1 >=0 and theta_0 >= 0) or (theta_1 <= 0 and theta_0 <= 0)):
		print "entered first case"
		if delta > 0:
			print "need to go left"
			direction = -1
			anglec = abs(delta)
		elif delta <0:
			print "need to go right"
			direction = 1
			anglec = abs(delta)
		else:
			direction = 0
			anglec = 0
		print "and angle", anglec
	else:	
		print "entered second case"
		if delta < 0:
			print "first within second"
			if delta > -180:
				print "going right"
				direction = 1
				anglec = abs(delta)
			else:
				print "going left"
				direction = -1
				anglec = 360 - abs(delta)
		elif delta > 0:
			print "second within second"
			if delta < 180:
				print "going left"
				direction = -1
				anglec = abs(delta)
			else:
				print "going right"
				direction = 1
				anglec = 360 - delta
		else:
			print "no rotation"
			direction = 0
			anglec = 0
		print "angle is ", anglec
	time.sleep(1)
		
	iyaw = find_yaw()
	print 'anglec ',anglec
	while abs(ang_diff(find_yaw(), iyaw) - anglec) > 5:
		move(anglec,direction)
	time.sleep(1)

	# [select, distance, angle, direction( left = 0 and right = 1), , ]
	idistance = find_distance()
	distance = distance_per_pixel*math.sqrt((source[1]-destination[1])**2 + (source[0]-destination[0])**2)
	dis_move([1,40,0,0,0,0])
	print 'Moving'
	while(dis_diff(find_distance(),idistance)) < (distance/100.0) -0.2:		
		print "in loop"	
		continue
	dis_move([-1,0,0,0,0,0])
	print 'Stopping command'
	time.sleep(1)

	theta_0 = theta_1
