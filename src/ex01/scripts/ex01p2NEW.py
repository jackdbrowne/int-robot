#! /usr/bin/env python
import numpy as np, numpy.ma as ma, rospy, random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

tolerance = 0.3
angle_mul = 0.1
linear_mul = 0.15
linear_dist = 0.5
rarray = [True, True, False]

def in_range(array,sensor_data):
#	for reading in array:
#		if reading < 0.6:
#			reading = 0.6
#		elif reading > 4.095:
#			reading = 4.095
	
	for i in range(0,len(array)-1):
		if array[i] < 0.6:
			array[i] = 0.6
		elif array[i] > 4.095:
			array[i] = 4.095
	
	array[np.isinf(array)]=sensor_data.range_max
	return array

def remove(reading, range_min, range_max):
	arr = []
	
	if reading < range_min or reading > range_max or np.isnan(reading):
		return True
	else:
		return False

def callback( sensor_data ):
	#sensor_data (LaserScan data type) has the laser scanner data
	#base_data (Twist data type) created to control the base
	
	scans = np.array_split(sensor_data.ranges, 3)
	
	left = ma.masked_invalid(scans[0]).filled(sensor_data.range_max)
	left_avg = np.mean(left)
	
	middle = ma.masked_invalid(scans[1]).filled(sensor_data.range_max)
	middle_avg = np.mean(middle)
	
	right = ma.masked_invalid(scans[2]).filled(sensor_data.range_max)
	right_avg = np.mean(right)
	

	
	#print("Right: " + str(right) + ", Middle: " + str(middle) + ", Left: " + str(left))
	print("Right_avg: " + str(right_avg) + ", Middle_avg: " + str(middle_avg) + ", Left_avg: " + str(left_avg))
	
#	print("Scans: " + str(scans[0]))
#	print("Ratio: "+ str(angle_ratio) + ", Angle: " + str(angle) + ", Forward: " + str(forward))
	print("-------------------------------------------------------")
	
if __name__ == '__main__':
    rospy.init_node('reactive_mover_node')
    rospy.Subscriber('base_scan', LaserScan, callback)
    rospy.spin()
