#! /usr/bin/env python
import rospy
import numpy as np, numpy.ma as ma, rospy,  random
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

def callback( odom_data ):
	global sleeptime
	#global p, start, end
	twist.angular.z = 0.5
	
	if sleeptime is None :
		print("timer not set")
		if odom_data.twist.twist.angular.z == 0:
			pub.publish(twist)
			print ("trying to move")
		else:
			sleeptime = rospy.get_rostime() + rospy.Duration.from_sec(16)
			#start = p
			moving = True
			print("timer set!")
	else :
		#end = p
		print("already set")
	
def scan_filter(arr, fill_val):
	""" Filters out invalid scan data from a given array of ranges
	
	Args:
		arr: the array of scan data that should be filtered
		fill_val: the value to replace 'inf' values with
		
	Returns:
		A copy of the input array, without 'nan' values and with 'inf' values replaces by the given fill_val
	"""
	arr = ma.masked_where(np.isnan(arr), arr).compressed()
	arr = ma.masked_where(np.isinf(arr), arr).filled(fill_val)
	
	return arr	
'''	
def lasercallback( laser_data ):
	global p
	scans = np.array_split(laser_data.ranges, 3)
	
	right = scan_filter(scans[0], laser_data.range_max)
	right_avg = np.median(right)
	
	left = scan_filter(scans[2], laser_data.range_max)
	left_avg = np.median(left)
	
	p = [right_avg,left_avg]
	
	print p
'''
if __name__ == '__main__':
	global sleeptime
	sleeptime = None
	#global p, start, end
	#p = []
	rospy.init_node('mover_node')
	pub = rospy.Publisher('cmd_vel', Twist)
	#start = []
	#end = []
	rospy.Subscriber('odom', Odometry, callback)
	#rospy.Subscriber('base_scan', LaserScan, lasercallback)
	twist = Twist()
	
	
	while not rospy.is_shutdown():
		if not sleeptime is None:
			if rospy.get_rostime() < sleeptime:
				twist.angular.z = math.pi/8
			else :
				twist.angular.z = 0
			pub.publish(twist)
	
	#print(str(start) + " :: " + str(end))
