#! /usr/bin/env python
from random import uniform
import numpy

import roslib
import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import (Twist, PoseArray)

import actionlib
import call_a_meeting.msg

class LocalisingAction(object):
    # create messages that are used to publish feedback/result
    _feedback = call_a_meeting.msg.LocaliseFeedback()
    _result = call_a_meeting.msg.LocaliseResult()
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, call_a_meeting.msg.LocaliseAction, auto_start = False)
        self._as.register_goal_callback(self.goal_cb)
        self._as.register_preempt_callback(self.preempt_cb)
        
        self._scan_sub = rospy.Subscriber('base_scan', LaserScan, self.scan_cb)
        self._cloud_sub = rospy.Subscriber('particlecloud', PoseArray, self.amcl_cb)
        self._pub = rospy.Publisher('cmd_vel', Twist)
        self._as.start()
    
    def goal_cb(self):
        self._goal = self._as.accept_new_goal()
        
    def preempt_cb(self):
        rospy.loginfo("%s: Preempted" % self._action_name)
        self._as.set_preempted()
        
    def scan_cb(self, laser_scan):
    	if not self._as.is_active():
    		return
        middle = numpy.array_split(laser_scan.ranges, 3)[1]
        move = Twist()
       	if min(middle) > 0.5:
	        move.linear.x = 0.25
	        move.angular.z = 0
       	else:
	    	move.linear.x = 0
	    	move.angular.z = uniform(-1.0, 1.0)
      	self._pub.publish(move)
      	
    def amcl_cb(self, pose_array):
    	if not self._as.is_active():
    		return
    	pose_count = len(pose_array.poses)
    	
    	self._feedback.count = pose_count
    	self._as.publish_feedback(self._feedback)
    	
    	if pose_count < 1000:
    		rospy.loginfo("Robot localised")
    		self._as.set_succeeded(self._result)

def LocaliserClient():
    ac = actionlib.SimpleActionClient('localiser', call_a_meeting.msg.LocaliseAction)
    
    rospy.loginfo("Waiting for connection to \'\\localiser\' server")
    ac.wait_for_server()
    
    rospy.loginfo("Creating LocaliseGoal message")
    goal = call_a_meeting.msg.LocaliseGoal()
    
    rospy.loginfo("Sending goal to server")
    ac.send_goal(goal)
    
    rospy.loginfo("Waiting for server response")
    ac.wait_for_result()
    
    return ac.get_result()
      
if __name__ == '__main__':
    rospy.init_node('localiser')
    rospy.loginfo("Starting up %s server" % rospy.get_name())
    LocalisingAction(rospy.get_name())
    rospy.spin()
