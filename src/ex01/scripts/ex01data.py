#! /usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

tolerance = 0.3
ratio_mul = 20

def callback( sensor_data ):
	#sensor_data (LaserScan data type) has the laser scanner data
	#base_data (Twist data type) created to control the base
	
	third = len(sensor_data.ranges)/3
	left = np.array([sensor_data.ranges[i] for i in range(0,third) if not np.isnan(sensor_data.ranges[i])])
	left[np.isinf(left)]=6
	left_avg = np.mean(left)
	
	middle = np.array([sensor_data.ranges[i] for i in range(third,third*2) if not np.isnan(sensor_data.ranges[i])])
	middle[np.isinf(middle)]=6
	middle_avg = np.mean(middle)
	
	right = np.array([sensor_data.ranges[i] for i in range(third*2, len(sensor_data.ranges)-1) if not np.isnan(sensor_data.ranges[i])])
	right[np.isinf(right)]=6
	right_avg = np.mean(right)
	
	print ("Right: " + str(right_avg))
	
	base_data = Twist()
	
	base_data.angular.z = 5
	
	pub.publish( base_data  )

if __name__ == '__main__':
    rospy.init_node('reactive_mover_node')
    rospy.Subscriber('base_scan', LaserScan, callback)
    pub = rospy.Publisher('cmd_vel', Twist)
    rospy.spin()
