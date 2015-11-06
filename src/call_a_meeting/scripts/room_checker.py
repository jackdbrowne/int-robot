#! /usr/bin/env python

import roslib
import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import (Twist, PoseArray)

import actionlib
import call_a_meeting.msg

from math import pi
import time

class CheckingRoomAction(object):
    # create messages that are used to publish feedback/result
    _feedback = call_a_meeting.msg.CheckRoomFeedback()
    _result = call_a_meeting.msg.CheckRoomResult()
    

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, call_a_meeting.msg.CheckRoomAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.register_preempt_callback(self.preempt_cb)
        
        self._scan_sub = rospy.Subscriber('leg_detector_topic', PoseArray, self.leg_cb)
        self._pub = rospy.Publisher('cmd_vel', Twist)
        self._as.start()
    
    def execute_cb(self, goal):
    	print "hi"
    	self._readings = 0.0
        self._leg_count = 0.0

        sleeptime = rospy.get_rostime() + rospy.Duration.from_sec(16)
        twist = Twist()
        twist.angular.z = pi/8
        while rospy.get_rostime() < sleeptime:
            rospy.get_rostime()
            self._pub.publish(twist)

        self._result.count = self._feedback.count
        if self._result.count <= goal.base_reading:
            self._as.set_succeeded(self._result)
        else:
            self._as.set_aborted(self._result)
        
    def preempt_cb(self):
        rospy.loginfo("%s: Preempted" % self._action_name)
        self._as.set_preempted()      
        
    def leg_cb(self, legs):
    	if not self._as.is_active():
            return
        self._readings += 1
        self._leg_count += len(legs.poses)
        self._feedback.count = self._leg_count/self._readings
        self._as.publish_feedback(self._feedback)

def CheckingRoomClient():
    ac = actionlib.SimpleActionClient('room_checker', call_a_meeting.msg.CheckRoomAction)
    
    rospy.loginfo("Waiting for connection to \'\\room_checker\' server")
    ac.wait_for_server()
    
    rospy.loginfo("Creating CheckRoomGoal message")
    goal = call_a_meeting.msg.CheckRoomGoal(base_reading=1)
    
    rospy.loginfo("Sending goal to server")
    ac.send_goal(goal)
    
    rospy.loginfo("Waiting for server response")
    ac.wait_for_result()
    
    return ac.get_result()
      
if __name__ == '__main__':
    rospy.init_node('room_checker')
    rospy.loginfo("Starting up %s server" % rospy.get_name())
    CheckingRoomAction(rospy.get_name())
    # CheckingRoomClient()
    rospy.spin()
