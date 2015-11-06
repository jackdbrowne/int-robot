#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import message_filters
import csv
from pf_localisation.util import getHeading
from datetime import datetime
from os.path import exists, expanduser
from time import time as time 
from math import hypot

def callback(odom_data, est_data):
	odom_pose = odom_data.pose.pose
	odom_position = odom_pose.position
        odom_orientation = odom_pose.orientation
	est_pose = est_data.pose
	est_position = est_pose.position
        est_orientation = est_pose.orientation
	timeSTAMP = str(odom_data.header.stamp.secs) + "." + str(odom_data.header.stamp.nsecs)
	
        if time() < sleeptime:
        	print("Writing")
                p.writerow([timeSTAMP, hypot(odom_position.x - (est_position.x - x_trans), odom_position.y - (est_position.y - y_trans))])
                o.writerow([timeSTAMP, getHeading(odom_orientation) - getHeading(est_orientation)])
        else :
                print("Experiment finished")
	
if __name__ == '__main__':
	rospy.init_node('experimentation_estpose')

	#size of map, taken from lgfloor.world
	x_trans = 33.1 / 2
	y_trans = 31.95 / 2

        odom_sub = message_filters.Subscriber('base_pose_ground_truth', Odometry)
	est_sub = message_filters.Subscriber('estimatedpose',PoseStamped)
       	ts = message_filters.TimeSynchronizer([odom_sub,est_sub],10)

	date = datetime.now().strftime("%Y%m%d-%H%M")
	path = expanduser('~/robotics/src/experiments/localization/amcl_mcl')
        with open('%s/position/%s.csv' % (path, date),'w') as pos_writer, open('%s/orientation/%s.csv' % (path, date),'w') as orient_writer: 
                p = csv.writer(pos_writer)
                o = csv.writer(orient_writer)
                p.writerow(['time', 'difference'])
                o.writerow(['time', 'difference'])
	
                sleeptime = time() + 300
                print("Beginning experiment")
                print("Do not interrupt")
                ts.registerCallback(callback)

                rospy.spin()
