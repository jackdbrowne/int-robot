#!/usr/bin/env python
import roslib
import rospy
import smach
import smach_ros

from geometry_msgs.msg import PoseArray

class bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['bar_succeeded'])
    def execute(self, userdata):
        rospy.loginfo("Found a person")
        return 'bar_succeeded'

def monitor_cb(ud, msg):
	if len(msg.poses) > 0:
		return False
	else:
		return True

def main():
    rospy.init_node("monitor_example")

    sm = smach.StateMachine(outcomes=['DONE'])
    with sm:
        smach.StateMachine.add('FOO', smach_ros.MonitorState("/leg_detector_topic", PoseArray, monitor_cb), transitions={'invalid':'BAR', 'valid':'FOO', 'preempted':'FOO'})
        smach.StateMachine.add('BAR',bar(), transitions={'bar_succeeded':'FOO'})

    sm.execute()
    rospy.spin()

if __name__=="__main__":
    main()
