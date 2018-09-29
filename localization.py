#!/usr/bin/env python
import rospy 
import time, math, random
from std_msgs.msg import String, Int32, Float32, String
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np
from robot_class import robot
from std_msgs.msg import Int16MultiArray

mapimage = cv2.imread('map4.pgm')


depth = ()
arduino_ack = 0
vals = []
orient = math.pi/2
pub=rospy.Publisher('array_values',Int16MultiArray,queue_size = 10)
rospy.init_node('pub_marray')
mainloop = 0


def scan_callback(scan):
    global depth
    depth = scan.ranges[:]
    return

def scan_listener():
    rospy.Subscriber("/scan", LaserScan, ack_callback)
    msg = rospy.wait_for_message('/scan', LaserScan)    
    print "this is the recieved scan"
    print msg
    return msg.ranges[:]

def ack_callback(data):
    global arduino_ack
    arduino_ack = data
    rospy.loginfo("received")
    return

def ack_listener():
    print "waiting for ack"
    rospy.Subscriber("/acknowledge", Int16MultiArray, ack_callback)
    return rospy.wait_for_message('/acknowledge', Int16MultiArray)

def senseWorld():
    for i in range(0, 4):
	time.sleep(1)
	vals.append(depth)
	print depth
	time.sleep(1)
	pub_data = [1,0,0,0,0,0]
        m=Int16MultiArray(data=pub_data)
        rospy.loginfo(pub_data)
        pub.publish(m)
        ack_listener()
    return vals


N = 250
particles = []
img1 = mapimage.copy()

for i in range(0, N):
	particles.append(robot())
	cv2.circle(img1, (particles[i].x, particles[i].y), 2, [0,255,0], -1)


print "created particles"
cv2.imshow('window', img1)
cv2.waitKey(0)

while mainloop <2 or particles.length == 0:

	vals = senseWorld()
	print vals

	weights = []
	count = 0
	img2 = mapimage.copy()
	for particle in particles:
		count+=1
		print "calculated weight", count
		yu = particle.measurement_prob(vals)
		weights.append(yu)
		if yu > 1.0e-340:
			cv2.circle(img2, (particle.x, particle.y), 2, [0,255,0], -1)
	
	cv2.imshow('2nd one', img2)
	cv2.waitKey(0)
	print weights
	w1 = weights[:]
	w1.sort(reverse=True)
	p1 = []
	mw = max(weights)
	index = int(random.random()*N)
	beta =0.0
	for i in range(N):
	    beta+=random.random()*2*mw
	    while beta > weights[index]:
		beta-= weights[index]
		index = (index+1)%N
	    p1.append(particles[index])
	
	img3 = mapimage
	for part in p1:
		cv2.circle(img3, (part.x, part.y), 2, [0,255,0], -1)

	cv2.imshow('filtered', img3)
	cv2.waitKey(0)
	particles = p1
	

