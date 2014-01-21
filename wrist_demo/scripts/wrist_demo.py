#!/usr/bin/env python

import roslib; roslib.load_manifest('wrist_demo')
import rospy
from hubomsg.msg import *
import sys
import select

class wrist_demo:

    def __init__( self ):

        rospy.init_node("face_center_controller")
        rospy.Subscriber("Maestro/Message", MaestroMessage, self.update)
        self.pub = rospy.Publisher("Maestro/Control", PythonMessage)
       
        self.LOW_THRESHOLD = -8

        self.MID_LOW_THRESHOLD = -25
        self.MID_HIGH_THRESHOLD = -40

        self.HIGH_THRESHOLD = -55

        self.RSP_EXTENDED = -1.38
        self.REP_EXTENDED = 0

        self.RSP_RETRACTED = 0
        self.REP_RETRACTED = -1.34


        self.OUT = 0
        self.STAY = 1
        self.IN = 2

        self.sleepVal = .01
        
        self.RSP_pos = 0
        self.REP_pos = 0
        self.RWT_val = 0
        self.last = self.OUT

    def setOut(self):
        self.pub.publish("RSP REP", "position position", str(self.RSP_EXTENDED) + " " + str(self.REP_EXTENDED), "")
        self.last = self.OUT
        rospy.sleep(self.sleepVal)

    def setStay(self):
        self.pub.publish("RSP REP","position position", str(self.RSP_pos) + " " + str(self.REP_pos), "")
        self.last = self.STAY
        rospy.sleep(self.sleepVal)

    def setIn(self):
        self.pub.publish("RSP REP","position position", str(self.RSP_RETRACTED) + " " + str(self.REP_RETRACTED), "")
        self.last = self.IN
        rospy.sleep(self.sleepVal)


    def update(self, message):
        if message.joint == "RSP" and message.property == "position":
            self.RSP_pos = message.value
        elif message.joint == "REP" and message.property == "position":
            self.REP_pos = message.value
        elif message.joint == "RWT" and message.property == "f_z":
            self.RWT_val = message.value
            
            if self.RWT_val < self.HIGH_THRESHOLD:
                self.setIn()        
            elif self.last == self.IN and self.RWT_val > self.MID_HIGH_THRESHOLD:
                self.setStay()
            elif self.last == self.OUT and self.RWT_val < self.MID_LOW_THRESHOLD:
                self.setStay()
            elif self.RWT_val > self.LOW_THRESHOLD:
                self.setOut()
   

if __name__ == '__main__':
    print "Starting the wrist demo"

    demo = wrist_demo()
    while not rospy.is_shutdown():
        demo.pub.publish("RSP","Get","0","position")
        rospy.sleep(demo.sleepVal)
        demo.pub.publish("REP","Get","0","position")
        rospy.sleep(demo.sleepVal)
        demo.pub.publish("RWT","Get","0","f_z")
        rospy.sleep(demo.sleepVal)
        
        rlist, wlist, elist = select.select([sys.stdin], [], [], 0)
        if rlist:
            rospy.sleep(1)
            demo.pub.publish("RSP REP","position position", "0 0", "")
            rospy.sleep(demo.sleepVal)
            sys.exit(0)

     
