#! /usr/bin/env python
import numpy as np, rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def callback( sensor_data ):
	#sensor_data (LaserScan data type) has the laser scanner data
	#base_data (Twist data type) created to control the base 
	
	#get the middle 1/3 of the sensor readings
	scans = np.array_split(sensor_data.ranges, 3)	
	middle = scans[1]

	base_data = Twist()
	
	
	#	move at 0.3 unless there is something 1m in front of robot, then stop
	if np.median(middle) > 1 :		
		base_data.linear.x = 0.3
	else :
		base_data.linear.x = 0
	

	# proportionaly calculate speed based distance of object from robot
	# limit this forward speed at 2, to avoid the robot speeding out of control
	#base_data.linear.x = min(2, np.median(middle) - 1)

	pub.publish( base_data  )

if __name__ == '__main__':
    rospy.init_node('reactive_mover_node')
    rospy.Subscriber('base_scan', LaserScan, callback)
    pub = rospy.Publisher('cmd_vel', Twist)
    rospy.spin()
