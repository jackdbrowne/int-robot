#!/usr/bin/env python

import sys
import rospy
from call_a_meeting.srv import *

def ask_client():
    rospy.wait_for_service('ask')
    try:
        ask = rospy.ServiceProxy('ask',Ask)
        resp1 = ask()
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
	print "start"
	answer = ask_client()
	if answer.answer:
		print "YAY YOU'RE COMING"
	else:
		print "w, myb nxt tm"
