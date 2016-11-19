#!/usr/bin/env python

#for the twist code: http://wiki.ros.org/mini_max/Tutorials/Moving%20the%20Base
#the general structure of this was taken from the tutorial on wiritng ROS publishers
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import  Clock
from nav_msgs.msg import Odometry
import random
from numpy import *





def get_pos_and_orient(msg,namer):
	global z_orient
	z_orient=msg.pose.pose.orientation.z

	global x_pos
	x_pos=round(msg.pose.pose.position.x,1)

	global y_pos
	y_pos=round(msg.pose.pose.position.y,1)

	#broadcast our position to the world
	# br = tf.TransformBroadcaster()

	
	# br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
	# 	(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w),
	# 				 # #tf.transformations.quaternion_from_euler(0,0,msg.pose.pose.orientation.z),
	# 				 rospy.Time.now(),
	# 				 namer,
	# 				 "world")

	# br.sendTransform((4.5,9,0),
	# 				 (0.,0.,0.,0.),
	# 				 # #tf.transformations.quaternion_from_euler(0,0,msg.pose.pose.orientation.z),
	# 				 rospy.Time.now(),
	# 				 'finish_point',
	# 				 "world")



def controller():

	rospy.init_node('controller',anonymous=True)
	# rospy.init_node('finish_point',anonymous=True)
	#defining the scans
	# listener = tf.TransformListener()
	global scan
	scan=LaserScan()

	#define the twist
	global twist
	twist=Twist()

	#define the collision variable
	global collisionDetect
	collisionDetect=5#for 3 seconds

	#creating the publisher
	global pub
	pub = rospy.Publisher('cmd_vel',Twist,queue_size=10)

	
	global direction
	direction=Odometry()



	global rate
	rate=rospy.Rate(100)

	global rand_direction

	global x_pos
	global y_pos


	#next is to setup the line which will basically be an array of points
	x_vals=linspace(-10,10,1000)
	x_vals=around(x_vals,decimals=1)
	# print x_vals
	y_vals=(11./12.5*x_vals+5.04)
	y_vals=around(y_vals,decimals=1)
	# print y_vals
	#print the goal point
	print x_vals[726], y_vals[726]

	while not rospy.is_shutdown():	#realtime control of the robot
		global final_x
		final_x=4.5
		global final_y
		final_y=9.



		try: 
			if((abs(x_pos-final_x)<0.2) and (abs(y_pos-final_y)<0.2)):#end condition
				print "finished!"
				# rospy.loginfo("Final coordinates: : "+str(x_pos)+" Y: "+str(y_pos))
				rospy.sleep(6.)

			elif((y_pos>round((11./12.5*x_pos+5.04),1)-0.3) and (y_pos<round((11./12.5*x_pos+5.04),1)+0.3) and (min(scan.ranges[175:185])>1.)):#we're on the line, and theres no obstical
				# rospy.loginfo("Line data: x: "+str(x_pos)+" Y: "+str(y_pos)+" Y should be: "+str(round(11./12.5*x_pos+5.04,1)))
				# rospy.loginfo("Z_orient: "+str(z_orient))
				rospy.loginfo(str(min((scan.ranges[165:195]))))
				go_along_line()
				
				# rospy.loginfo(scan.ranges[:10])
			elif((min(scan.ranges[170:190])>1.8) and (min(scan.ranges[270:360])<=1.5)):#some condition for wall following: its not facing a wall and its left side is close to the wall
				wall_follow()

			elif(min(scan.ranges[170:190])<=2.0):#turn and do some wall following
				rospy.loginfo("turning!")
				turn()
			else:
				# i=0
				rospy.loginfo("Off line: x: "+str(x_pos)+" Y: "+str(y_pos)+" Y should be: "+str(round(11./12.5*x_pos+5.04,1)))
				# rospy.loginfo(min(scan.ranges[:90]))			
			
		except:
			rospy.sleep(0.1)

def go_along_line():
	rospy.loginfo("on the line!")

	global final_x
	global final_y
	global x_pos
	global y_pos
	global pub

	trans=array([final_x-x_pos,final_y-y_pos])
	
	
	if((z_orient-0.35)>0.03):
		linear=0.
		angular=-1.
		# rospy.loginfo("Too high")
		# rospy.loginfo("Z_orient: "+str(z_orient))
	elif((0.35-z_orient)>0.03):
		linear=0.5
		angular=1.
		# rospy.loginfo("Too low!")
	else:
		angular = 4 * math.atan2(trans[1], trans[0])*3.1415/180.
		linear = 2.#0.5 * math.sqrt(trans[0]**2 + trans[1]**2)
		# rospy.loginfo("Angular: "+str(angular))
	cmd=Twist()
	cmd.linear.x = linear
	cmd.angular.z = angular
	rospy.loginfo(cmd.angular.z)
	pub.publish(cmd)
	rate.sleep()

def wall_follow():
	# rospy.loginfo("wall following!")
	#start here, you can't have the robot getting too close to the wall, so there must be some method in here to make is keep a cushion
	global pub
	cmd=Twist()
	cmd.linear.x = 2.
	if(sum(scan.ranges[270:360])<60):
		cmd.angular.z=-3.
	else:
		cmd.angular.z=3.
	# cmd.angular.z = 0
	# rospy.loginfo(cmd.angular.z)
	pub.publish(cmd)
	rate.sleep()


def turn():
	twist.linear.x=0
	twist.angular.z=-3
	pub.publish(twist)
	rate.sleep()

def angle_at(msg):
	global direction
	direction=msg
	#rospy.loginfo(msg.pose.pose.position.x)


def callback(msg):

	#start below
	global scan
	scan=msg

if __name__ == '__main__':
	try:
		#creating the subscriber for laser data
		rospy.Subscriber('base_scan',LaserScan,callback)
		rospy.Subscriber('/base_pose_ground_truth',Odometry,get_pos_and_orient,'/base_pose_ground_truth')

		#create the scan for odometry data
		rospy.Subscriber('odom',Odometry,angle_at)
		controller()
	except rospy.ROSInterruptException:
		pass