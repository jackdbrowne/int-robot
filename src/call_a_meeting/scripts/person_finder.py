#! /usr/bin/env python

import roslib
import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import (Twist, PoseArray)

import actionlib
import call_a_meeting.msg

from os.path import exists, expanduser
import cv2
import sys
import math

import numpy as np, numpy.ma as ma

class FindPersonAction(object):   


    def __init__(self, name):
        # set up the webcam and everything 
        print exists(expanduser('~/robotics/src/call_a_meeting/src/haarcascade_upperbody.xml'))
        self.faceCascade = cv2.CascadeClassifier(expanduser('~/robotics/src/call_a_meeting/src/haarcascade_upperbody.xml'))

        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, call_a_meeting.msg.FindPersonAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.register_preempt_callback(self.preempt_cb)
        
        self._scan_sub = rospy.Subscriber('/base_scan', LaserScan, self.person_cb)
        self._pub = rospy.Publisher('cmd_vel', Twist)
        self._as.start()
        print "done"

    # moving towards the person
    def moveTowards(self):

        point = 20

        while(point > 0.75):
            scans = np.array_split(self.laserscan.ranges, 5)
            middle = scans[3]
            middle = ma.masked_where(np.isnan(middle), middle).compressed()
            middle = ma.masked_where(np.isinf(middle), middle).filled(self.laserscan.range_max)

            point = min(middle)

            move = Twist();
            move.linear.x = 0.2
            move.angular.z = 0
            self._pub.publish(move)

        move = Twist();
        move.linear.x = 0
        move.angular.z = 2
        self._pub.publish(move)


    def execute_cb(self, goal):
        # here we want to detect a human every 1 sec
        print "hello"
        self.laserscan = None
        sleeptime = rospy.get_rostime() + rospy.Duration.from_sec(16)

        self.video_capture = cv2.VideoCapture(1)
        self.webcam_width = self.video_capture.get(3) # I don't know if we need it or not it is just incase

        while rospy.get_rostime() < sleeptime and self._as.is_active():
            print "Getting a photo"
            ret, frame = self.video_capture.read()

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            faces = self.faceCascade.detectMultiScale(
                gray,
                scaleFactor=1.2,
                minNeighbors=5,
                minSize=(64,64),
                # maxSize=(180,180),
                flags=cv2.cv.CV_HAAR_SCALE_IMAGE
            )

            for (x, y, w, h) in faces:
                print "Width" + str(w)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

            cv2.imshow('Video', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            print len(faces)

            if len(faces) > 0:
                move = Twist()
                move.linear.x = 0
                move.angular.z = 0
                self._pub.publish(move)

                self._as.set_succeeded()
                self.video_capture.release()
                return
            else:
                move = Twist()
                move.angular.z = math.pi/8
                self._pub.publish(move)

        self.video_capture.release()
        self._as.set_aborted()


        '''
        move = Twist()
        if len(faces) > 0:
            # is box in middle

                        # x coord    +  width
            middle_box = faces[0][0] + (faces[0][1] / 2)
            middle_cam = self.webcam_width / 2
            diff =  middle_box - middle_cam
            print diff

            if math.fabs(diff) < 50:
                # moved
                move.linear.x = 0
                move.angular.z = 0
                _ispeople = True
                self._pub.publish(move)

                self.moveTowards()

                print "FOUND SOMEONE"
            else:
                #try to move in the middle
                print "move to middle"
                move.linear.x = 0
                if diff < 0:
                    print "left"
                    move.angular.z = 0.1
                else:
                    print "right"
                    move.angular.z = -0.1  

                self._pub.publish(move)

        else:
            
            move.linear.x = 0
            move.angular.z = 0.25
            self._pub.publish(move)
        '''


        self._as.set_succeeded()




        
    def preempt_cb(self):
        rospy.loginfo("%s: Preempted" % self._action_name)
        self._as.set_preempted()      
        
    def person_cb(self, laser):

        #here we do the random movement or moving to a person
        if not self._as.is_active():
            return
        self.laserscan = laser
        # move = Twist()
        # move.linear.x = 0
        # move.angular.z = 0.25
        # self._pub.publish(move)

def FindPersonClient():
    ac = actionlib.SimpleActionClient('person_finder', call_a_meeting.msg.FindPersonAction)
    
    rospy.loginfo("Waiting for connection to \'\\person_finder\' server")
    ac.wait_for_server()
    
    rospy.loginfo("Creating FindPerson message")
    goal = call_a_meeting.msg.FindPersonGoal()
    
    rospy.loginfo("Sending goal to server")
    ac.send_goal(goal)
    
    rospy.loginfo("Waiting for server response")
    ac.wait_for_result()
    
    return ac.get_result()

if __name__ == '__main__':
    rospy.init_node('person_finder')
    rospy.loginfo("Starting up %s server" % rospy.get_name())
    FindPersonAction(rospy.get_name())
#    FindPersonClient()
    rospy.spin()
