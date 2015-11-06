#! /usr/bin/env python
import numpy as np, numpy.ma as ma, rospy,  random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from p2os_driver.msg import SonarArray

tolerance = 0.3	
angle_mul = 0.1
linear_mul = 0.15
linear_dist = 0.5
rand_factor = [True, False, False, False]

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
	
def sonar_callback( sensor_data ):
	if min(sensor_data.ranges) < 1 :
		base_data = Twist()
		base_data.linear.x = 0
		pub.publish( base_data )
		print("Sonar")
			
	
	
def callback( sensor_data ):
	#sensor_data (LaserScan data type) has the laser scanner data
	#base_data (Twist data type) created to control the base
	
	# split sensor readings into 5 equal parts (right, middle and left)
	scans = np.array_split(sensor_data.ranges, 5)
	
	#swapped left and right values, sensor reads RTL
	
	# filter invalid data from each split 
	right = scan_filter(scans[0], sensor_data.range_max)
	right_avg = np.median(right)
	
	right_middle = scan_filter(scans[1], sensor_data.range_max)
	right_middle_avg = np.median(right_middle)
	
	middle = scan_filter(scans[2], sensor_data.range_max)
	middle_avg = np.median(middle)
	
	left_middle = scan_filter(scans[3], sensor_data.range_max)
	left_middle_avg = np.median(left_middle)
	
	left = scan_filter(scans[4], sensor_data.range_max)
	left_avg = np.median(left)	

	base_data = Twist()
	
	# check for obstacles immediately in front of the robot
	# stop forward motion, turn a random amount 
	if min(min(right_middle), min(middle), min(left_middle)) < 0.7 or random.random() <= 0.05 :
		forward_ratio = -0
		angle_ratio = random.randint(-50, 50)
	# check for clear space of over 1.5m in all (measured) directions
	# also, randomly fail in 2/3 of the time (for random exploration)
	# TODO: the random won't turn very much because only uses diff(left, right) which will always be low; could possibly move into 		above statement with and 'or'
	elif min(left_avg, middle_avg, right_avg) > 1.5 :
		forward_ratio = middle_avg-linear_dist
		angle_ratio = 0
	# otherwise, turn proportionaly to the area with the most free space (left or right)
	else :
		forward_ratio = middle_avg-linear_dist
		angle_ratio = left_avg - right_avg
		
	angle = angle_ratio * angle_mul
	forward = forward_ratio * linear_mul
	print("Right: " + str(right_avg) + ", Left: " + str(left_avg))
	print("Ratio: "+ str(angle_ratio) + ", Angle: " + str(angle) + ", Forward: " + str(forward))
	print("-------------------------------------------------------")
	
	base_data.angular.z = angle
	base_data.linear.x = forward
	
	pub.publish( base_data  )

if __name__ == '__main__':
    rospy.init_node('reactive_mover_node')
    rospy.Subscriber('base_scan', LaserScan, callback)
  #  rospy.Subscriber('sonar', SonarArray, sonar_callback)
    
    pub = rospy.Publisher('cmd_vel', Twist)
    rospy.spin()
