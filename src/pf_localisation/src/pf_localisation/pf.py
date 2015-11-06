from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy

from util import rotateQuaternion, getHeading
from random import gauss, uniform, random
from numpy import cumsum, mean
import bisect

import subprocess, sys
from os.path import expanduser

from time import time

sys.path.append('/data/private/robot/robotics/src/ex01/scripts')
from ex01p2 import scan_filter

class PFLocaliser(PFLocaliserBase):

    def __init__(self):
        # Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # Set motion model parameters
 	self.ODOM_ROTATION_NOISE = 0.015 #
 	self.ODOM_TRANSLATION_NOISE = 0.04 #
 	self.ODOM_DRIFT_NOISE = 0.018 #
 
        # Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20 	# Number of readings to predict
        
        self.flag = False
        self.flag_two = False

        self.slow_decay = 0.0
        self.fast_decay = 0.0
       
    def initialise_particle_cloud(self, initialpose):
        # Set particle cloud to initialpose plus noise
        pose_arr = PoseArray()
        init = initialpose.pose.pose
        
        # using 200 particles
        for _ in range(0,200):
        	pose = Pose()
        	
        	# using gaussian distribution
        	pose.position.x = gauss(init.position.x,0.5)
 		pose.position.y = gauss(init.position.y,0.5)
 		rotation = gauss(0, 0.125)
 		pose.orientation = rotateQuaternion(init.orientation, rotation)
        	pose_arr.poses.append(pose)
        
 	return pose_arr
    
    def update_particle_cloud(self, scan):
    	#remove NaN and inf values
    	scan.ranges = scan_filter(scan.ranges, self.sensor_model.scan_range_max)
        
        if len(scan.ranges) > 0:
	        weights = [self.sensor_model.get_weight(scan, pose) for pose in self.particlecloud.poses]
	else:
		print "Error: Scan ranges null"
		return
    	
    	# used for finding how confident you are in the weights - decay the weight average
        w_avg = mean(weights)
        self.slow_decay += 0.001 * (w_avg - self.slow_decay)
        self.fast_decay += 0.1 * (w_avg - self.fast_decay)

        breakpoints=cumsum(weights)
        
    	maximum = max(breakpoints)
    	  
    	# the probability of the weights being okay
 	prob = max(0.0, 1.0 - (self.fast_decay/self.slow_decay))   	   
    	if not prob == 0:
	    	loops = int(len(self.particlecloud.poses) * prob)
    	else:
    		loops = 0

        # Update particlecloud, given map and laser scan
       	pose_arr = PoseArray()
        for i in range(0,len(self.particlecloud.poses)):
            new_pose = Pose()
            # add completely random noise to re-localise
            if i < loops:                                   
            	# 33.1 and 31.95 being the x and y sizes of the map respectively
                new_pose.position.x = uniform(0, 33.1)
                new_pose.position.y = uniform(0,31.95)
                new_pose.orientation = rotateQuaternion(Quaternion(w=1), uniform(0, math.pi*2)) 
            # otherwise use roulette wheel resampling to resample
            else:
            	# make a random pick
		pick = uniform(0, maximum)
		# an nlogn implementation of the roulette wheel search - by splitting sorted list in half repeatedly
		i = bisect.bisect(breakpoints, pick)
		choice = self.particlecloud.poses[i]
		position = choice.position
		orientation = choice.orientation
		
		# add resampling noise to cope with changes in odometry
		new_pose.position.x = gauss(position.x, 0.05)
		new_pose.position.y = gauss(position.y, 0.05)
		rotation = gauss(0, 0.125)
		new_pose.orientation = rotateQuaternion(orientation, rotation)
		new_pose.orientation = orientation
		
	    pose_arr.poses.append(new_pose)
	
	self.particlecloud = pose_arr		
        
    def estimate_pose(self):
        # Create new estimated pose, given particle cloud
        # E.g. just average the location and orientation values of each of
        # the particles and return this.
        
        # Better approximations could be made by doing some simple clustering,
        # e.g. taking the average location of half the particles after 
        # throwing away any which are outliers
       	
       	"""
       	best_pose = {}
       	for pose in self.particlecloud.poses:
       		pose_score = 0
       	
       		for others in self.particlecloud.poses:
       			pose_score += math.hypot(pose.position.x - others.position.x, pose.position.y - others.position.y)
        	
        	best_pose[pose] = pose_score
	
#	dict_keys = best_pose.keys()
#	dict_vals = best_pose.values()
	
#	for i in range(0,len(dict_keys)):
#		print("x: " + str(dict_keys[i].position.x) + " || y: " + str(dict_keys[i].position.y) + " || score: " + str(dict_vals[i]))
	
#       print(str(min(best_pose,key=best_pose.get)))
        
#        rtn = min(best_pose,key=best_pose.get)
        
        #print (str(rtn.position.x) + ", " + str(rtn.position.y))
        
        return min(best_pose,key=best_pose.get)
        """
        
        best_pose = {}
        # the distance in m we are checking within
        _range = 0.1
        
        for pose in self.particlecloud.poses:
        	# a score assigned to each pose to show it's quality
        	pose_score = 0
        	
        	posex = pose.position.x
        	posey = pose.position.y
        	
        	for others in self.particlecloud.poses:
        		othersx = others.position.x
        		othersy = others.position.y
        	
        		if othersx > posex - _range and othersx < posex + _range and othersy > posey - _range and othersy < posey + _range:
        			pose_score += 1

		# a dictionary of all scores and their pose_score        			
        	best_pose[pose] = pose_score
        	
        #return the pose with the highest pose_score	
       	return max(best_pose,key=best_pose.get)
       	
       	"""
       	av_x = 0
	av_y = 0
	av_head = 0
	
	for pose in self.particlecloud.poses:
		av_x += pose.position.x
		av_y += pose.position.y
		av_head += getHeading(pose.orientation)

	l = len(self.particlecloud.poses)

	av_pose = Pose()

	av_pose.position.x = av_x / l
	av_pose.position.y = av_y / l
	av_pose.orientation = rotateQuaternion(Quaternion(w=1),(av_head / l))
	
	return av_pose
	
	"""	
	"""
	l = len(self.particlecloud.poses)
	     
	avg_pose = Pose()
	avg_pose.position.x = sum(pose.position.x for pose in self.particlecloud.poses)/l
	avg_pose.position.y = sum(pose.position.y for pose in self.particlecloud.poses)/l
	avg_pose.orientation = rotateQuaternion(Quaternion(w=1), sum(getHeading(pose.orientation) for pose in self.particlecloud.poses)/l)
	     
	return avg_pose
	"""

