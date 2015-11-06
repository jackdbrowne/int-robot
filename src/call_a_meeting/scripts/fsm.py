#!/usr/bin/env python

## To run this node run the following commands
## roscore
## roslaunch 2dnav run_stage_nodes.launch (Don't set initialpose in RViz. Also add the maps and paths etc. to RViz)
## roslaunch 2dnav move_base_stage.launch
## rosrun call_a_meeting localiser.py
## rosrun call_a_meeting fsm.py
##
## Should probably write a launch file for all that!

import rospy, roslib
import smach, smach_ros

from call_a_meeting.msg import *
import std_srvs.srv

from move_base_msgs.msg import *
from geometry_msgs.msg import (Pose, PoseStamped, Point, Quaternion)
from std_msgs.msg import Header

import call_a_meeting.msg
import call_a_meeting.srv

header = Header(frame_id='map')
base_orient = Quaternion(w = 1)
room1_config = {'door_point': Point(15.8992752204, 15.555323894, 0.0),
                'middle_point': Point(12.9025080207, 16.6119487331, 0.0),
                'exit_orient': Quaternion(z=0.490140818277,w=0.871643263187),
                'entrance_orient': Quaternion( z=0.999640179281, w=0.0268237202182)}

room1 = {'entrance': PoseStamped(header=header,
                               pose=Pose(room1_config['door_point'], room1_config['entrance_orient'])),
         'middle': PoseStamped(header=header,
                               pose=Pose(room1_config['middle_point'], room1_config['entrance_orient'])),
         'exit': PoseStamped(header=header,
                               pose=Pose(room1_config['door_point'], room1_config['exit_orient']))}

room2_config = {'door_point': Point(19.94455, 10.48432, 0.0),
                'middle_point': Point(16.9, 12.6, 0.0),
                'exit_orient': Quaternion(x = 0.0, y = 0.0, z = 0.0909583217977, w = 0.995854700092),
                'entrance_orient': Quaternion(x = 0.0, y = 0.0, z = 0.965238807617, w = 0.261369554981)}

room2 = {'entrance': PoseStamped(header=header,
                               pose=Pose(room2_config['door_point'], room2_config['entrance_orient'])),
         'middle': PoseStamped(header=header,
                               pose=Pose(room2_config['middle_point'], room2_config['entrance_orient'])),
         'exit': PoseStamped(header=header,
                               pose=Pose(room2_config['door_point'], room2_config['exit_orient']))}

find_person_points = [PoseStamped(header = header, pose=Pose(Point(x=4.17562866211,y=19.7902183533),base_orient)),
                      PoseStamped(header = header, pose=Pose(Point(x=8.89898967743,y=22.4806156158),base_orient)),
                      PoseStamped(header = header, pose=Pose(Point(x=12.1135578156,y=27.073638916),base_orient)),
                      PoseStamped(header = header, pose=Pose(Point(x=13.5567789078,y=19.5932312012),base_orient)),
                      PoseStamped(header = header, pose=Pose(Point(x=17.8389217585,y=14.6013434945),base_orient)),
                      PoseStamped(header = header, pose=Pose(Point(x=20.4448795319,y=8.37262058258),base_orient)),
                      PoseStamped(header = header, pose=Pose(Point(x=20.6416816711,y=4.10747528076),base_orient)),
                      PoseStamped(header = header, pose=Pose(Point(x=23.1345176697,y=8.24138450623),base_orient))]

def _goal_cb(userdata, goal):
        print ("userdata.move.counter_out: %d" % (userdata.move_counter_in))
        _goal = MoveBaseGoal(target_pose=userdata.move_config[userdata.move_counter_in])
        if userdata.move_counter_in == 0:
            userdata.move_inc_out = True
        elif userdata.move_counter_in == len(userdata.move_config)-1:
            userdata.move_inc_out = False
        if userdata.move_inc_in:
            userdata.move_counter_out = userdata.move_counter_in + 1
        else:
            userdata.move_counter_out = userdata.move_counter_in - 1

        return _goal

def _ask_callback(userdata, response):
            if response.answer:
                return "succeeded"
            else:
                return "aborted"

def find_person(counter_val):

    sm = smach.StateMachine(outcomes=['succeeded'])
    sm.userdata.move_points = find_person_points
    sm.userdata.counter = counter_val
    sm.userdata.inc = True
    with sm:
        smach.StateMachine.add('MOVE',
                               smach_ros.SimpleActionState('move_base', MoveBaseAction, goal_cb=_goal_cb, 
                                input_keys=['move_config','move_counter_in','move_inc_in'],
                                output_keys=['move_counter_out','move_inc_out']),
                                transitions={'succeeded':'LOOK',
                                            'preempted':'MOVE',
                                            'aborted':'MOVE'},
                                remapping={'move_config':'move_points',
                                           'move_counter_in':'counter',
                                           'move_counter_out':'counter',
                                           'move_inc_in':'inc',
                                           'move_inc_out':'inc'})
        smach.StateMachine.add('LOOK',
                               smach_ros.SimpleActionState('person_finder', call_a_meeting.msg.FindPersonAction, goal=call_a_meeting.msg.FindPersonGoal()),
                               transitions={'succeeded':'ASK_QUESTION',
                                            'preempted':'MOVE',
                                            'aborted':'MOVE'})

        smach.StateMachine.add('ASK_QUESTION',
                               smach_ros.ServiceState('ask',call_a_meeting.srv.Ask,
                                                      response_cb = _ask_callback),
                               transitions={'succeeded':'succeeded', #Take person to meeting room
                                            'preempted': 'MOVE',
                                            'aborted': 'MOVE'},)  #Go back to movement/Find person

    return sm

def fill_room1():
    sm = smach.StateMachine(outcomes=['succeeded'])
    with sm:
        smach.StateMachine.add('FIND_PERSON_1', find_person(0),
                            transitions={'succeeded':'MOVE_1'})

        smach.StateMachine.add('MOVE_1',
                            smach_ros.SimpleActionState('move_base', MoveBaseAction, goal=MoveBaseGoal(target_pose=room1['entrance'])),
                            transitions={'succeeded':'FIND_PERSON_2',
                                         'preempted':'FIND_PERSON_1',
                                         'aborted':'FIND_PERSON_1'})

        smach.StateMachine.add('FIND_PERSON_2', find_person(0),
                            transitions={'succeeded':'MOVE_2'})

        smach.StateMachine.add('MOVE_2', 
                            smach_ros.SimpleActionState('move_base', MoveBaseAction, goal=MoveBaseGoal(target_pose=room1['entrance'])),
                            transitions={'succeeded':'succeeded',
                                         'preempted':'FIND_PERSON_1',
                                         'aborted':'FIND_PERSON_1'})
    return sm

def fill_room2():
    sm = smach.StateMachine(outcomes=['succeeded'])
    with sm:
        smach.StateMachine.add('FIND_PERSON_1', find_person(7),
                            transitions={'succeeded':'MOVE_1'})

        smach.StateMachine.add('MOVE_1',
                            smach_ros.SimpleActionState('move_base', MoveBaseAction, goal=MoveBaseGoal(target_pose=room2['entrance'])),
                            transitions={'succeeded':'FIND_PERSON_2',
                                         'preempted':'FIND_PERSON_1',
                                         'aborted':'FIND_PERSON_1'})

        smach.StateMachine.add('FIND_PERSON_2', find_person(7),
                            transitions={'succeeded':'MOVE_2'})

        smach.StateMachine.add('MOVE_2', 
                            smach_ros.SimpleActionState('move_base', MoveBaseAction, goal=MoveBaseGoal(target_pose=room2['entrance'])),
                            transitions={'succeeded':'succeeded',
                                         'preempted':'FIND_PERSON_1',
                                         'aborted':'FIND_PERSON_1'})
    return sm



def check_room1():
    sm = smach.StateMachine(outcomes=['full', 'empty'])
    with sm:
        smach.StateMachine.add('ENTER',
                               smach_ros.SimpleActionState('move_base', MoveBaseAction, goal=MoveBaseGoal(target_pose=room1['entrance'])),
                               transitions={'succeeded':'MOVE_MIDDLE',
                                            'preempted':'MOVE_MIDDLE',
                                            'aborted':'MOVE_MIDDLE'})

        smach.StateMachine.add('MOVE_MIDDLE',
                               smach_ros.SimpleActionState('move_base', MoveBaseAction, goal=MoveBaseGoal(target_pose=room1['middle'])),
                               transitions={'succeeded':'CHECK_1',
                                            'preempted':'FULL_EXIT',
                                            'aborted':'FULL_EXIT'})

        smach.StateMachine.add('CHECK_1',
                                   smach_ros.SimpleActionState('room_checker', call_a_meeting.msg.CheckRoomAction, goal=call_a_meeting.msg.CheckRoomGoal(base_reading=0.762219905853)),
                                   transitions={'succeeded':'CHECK_2',
                                                'preempted':'FULL_EXIT',
                                                'aborted':'FULL_EXIT'})

        smach.StateMachine.add('CHECK_2',
                                   smach_ros.SimpleActionState('room_checker', call_a_meeting.msg.CheckRoomAction, goal=call_a_meeting.msg.CheckRoomGoal(base_reading=0.762219905853)),
                                   transitions={'succeeded':'EMPTY_EXIT',
                                                'preempted':'FULL_EXIT',
                                                'aborted':'FULL_EXIT'})

        smach.StateMachine.add('FULL_EXIT',
                               smach_ros.SimpleActionState('move_base', MoveBaseAction, goal=MoveBaseGoal(target_pose=room1['exit'])),
                               transitions={'succeeded':'full',
                                            'preempted':'full',
                                            'aborted':'full'})

        smach.StateMachine.add('EMPTY_EXIT',
                               smach_ros.SimpleActionState('move_base', MoveBaseAction, goal=MoveBaseGoal(target_pose=room1['exit'])),
                               transitions={'succeeded':'empty',
                                            'preempted':'MOVE_MIDDLE',
                                            'aborted':'MOVE_MIDDLE'})
    return sm

def check_room2():
    header = Header(frame_id='map')
    
    sm = smach.StateMachine(outcomes=['full', 'empty'])
    with sm:
        smach.StateMachine.add('ENTER',
                               smach_ros.SimpleActionState('move_base', MoveBaseAction, goal=MoveBaseGoal(target_pose=room2['entrance'])),
                               transitions={'succeeded':'MOVE_MIDDLE',
                                            'preempted':'MOVE_MIDDLE',
                                            'aborted':'MOVE_MIDDLE'})

        smach.StateMachine.add('MOVE_MIDDLE',
                               smach_ros.SimpleActionState('move_base', MoveBaseAction, goal=MoveBaseGoal(target_pose=room2['middle'])),
                               transitions={'succeeded':'CHECK_1',
                                            'preempted':'FULL_EXIT',
                                            'aborted':'FULL_EXIT'})

        smach.StateMachine.add('CHECK_1',
                                   smach_ros.SimpleActionState('room_checker', call_a_meeting.msg.CheckRoomAction, goal=call_a_meeting.msg.CheckRoomGoal(base_reading=0.35)),
                                   transitions={'succeeded':'CHECK_2',
                                                'preempted':'FULL_EXIT',
                                                'aborted':'FULL_EXIT'})

        smach.StateMachine.add('CHECK_2',
                                   smach_ros.SimpleActionState('room_checker', call_a_meeting.msg.CheckRoomAction, goal=call_a_meeting.msg.CheckRoomGoal(base_reading=0.35)),
                                   transitions={'succeeded':'EMPTY_EXIT',
                                                'preempted':'FULL_EXIT',
                                                'aborted':'FULL_EXIT'})

        smach.StateMachine.add('FULL_EXIT',
                               smach_ros.SimpleActionState('move_base', MoveBaseAction, goal=MoveBaseGoal(target_pose=room2['exit'])),
                               transitions={'succeeded':'full',
                                            'preempted':'full',
                                            'aborted':'full'})

        smach.StateMachine.add('EMPTY_EXIT',
                               smach_ros.SimpleActionState('move_base', MoveBaseAction, goal=MoveBaseGoal(target_pose=room2['exit'])),
                               transitions={'succeeded':'empty',
                                            'preempted':'MOVE_MIDDLE',
                                            'aborted':'MOVE_MIDDLE'})
    return sm

def find_empty():
    # Use the same header for each Pose
    
    sm = smach.StateMachine(outcomes=['1', '2'])
    with sm:
        smach.StateMachine.add('ROOM1', check_room1(),
                               transitions={'full': 'ROOM2',
                                            'empty': '1'})
        smach.StateMachine.add('ROOM2', check_room2(),
                               transitions={'full': 'ROOM1',
                                            'empty': '2'})
    return sm


if __name__ == '__main__':
    rospy.init_node('state_machine')
    
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    
    with sm:
        smach.StateMachine.add('LOCALISE',
                               smach_ros.SimpleActionState('localiser', LocaliseAction),
                               transitions={'succeeded': 'FIND_EMPTY',
                                            'preempted': 'failed',
                                            'aborted': 'failed'})
        
        smach.StateMachine.add('FIND_EMPTY', find_empty(),
                               transitions={'1': 'FILL_1',
                                            '2': 'FILL_2'})
        
        smach.StateMachine.add('FILL_1', fill_room1(),
                                transitions={'succeeded':'succeeded'})

        smach.StateMachine.add('FILL_2', fill_room2(),
                                transitions={'succeeded':'succeeded'})
                                	
    sis = smach_ros.IntrospectionServer('statemachine_viewer', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()
