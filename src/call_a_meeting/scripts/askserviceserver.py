#!/usr/bin/env python

from call_a_meeting.srv import *
import rospy
import pyttsx
import threading
import sys
from select import select

def question():
    print "question()"
    engine.say("Hey! Would you like to go to a meeting?")
    engine.runAndWait()

def handle_ask(req):
    joker = False

    response = AskResponse()
    
    t = threading.Thread(target=question)
    t.start()

    while True:
    
        print "Would you like to come to to the meeting?"
        print "Press 'Y' for Yes and 'N' for No"
        print "Hit enter when you're done!"

        timeout = 10
        print "Enter something:",
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            input_char = sys.stdin.readline()
            print input_char
        else:
            print "No input. Moving on..."
            response.answer = False
            return response

        if input_char == 'y\n' or input_char == 'Y\n':
            print "YEAH BOY! FOLLOW ME!"
            response.answer = True
            return response
        elif input_char == 'n\n' or input_char == 'N\n':
            print "NO BOY!"
            response.answer = False
            return response
        elif joker:
            print "Stop it, you"
            response.answer = False
            return response
        else:
            print "Wrong input\n"
            joker = True

def ask_server():
    rospy.init_node('ask_server')
    s = rospy.Service('ask', Ask, handle_ask)
    
    global engine 
    engine = pyttsx.init()
    engine.setProperty('rate', 70)
    
    print "Ready to ask."
    rospy.spin()

if __name__ == "__main__":
    ask_server()