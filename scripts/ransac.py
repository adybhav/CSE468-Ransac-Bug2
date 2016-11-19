#!/usr/bin/env python
import  roslib
import rospy
from geometry_msgs.msg import Twist
import math
import numpy
import tf
import random
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


def drawer():
	rospy.Rate(10)
	line=Marker.LINE_LIST
	pub1=rospy.Publisher("visualization_marker",Marker, queue_size=10)
	pub2=rospy.Publisher("vis_actual",Marker,queue_size=10)
	rospy.Subscriber("/base_scan",LaserScan,callback)
	
def callback(data):
	lista =[]        
	points=[]		
	#global points
	point=Point()	
	for i in range(1,361):
		if(data.ranges[i]<3.0):
			lista.append(data.ranges[i])
	for i in lista:
		point.x=data.ranges[i]*math.cos(i*data.angle_increment)
		point.y=data.ranges[i]*math.sin(i*data.angle_increment)
		points.append(point)
	
	p1=random.choice(points)
	p2=random.choice(points)
	x1=p1.x
	x2=p2.x
	y1=p1.y
	y2=p2.y
	perm=[]	
	inlier=[]
	outlier=[]					
	for i in range(0,250):
		ran1=random.choice(points)
		x0=ran1.x
		y0=ran1.y
		dist = abs((y2 - y1) * x0 - (x2 - x1)*y0 + x2 * y1 - y2 * x1) / math.sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1))
		if(dist<.2):
			inlier.append(ran1)
		else:
			outlier.append(ran1)	
	if(len(inlier)>len(perm)):
		perm=inlier
		
if __name__ == '__main__':
    try:
        rospy.init_node('ransac', anonymous=True)
	point=Point()
	twist=Twist()
	drawer()
    except rospy.ROSInterruptException:
        pass
