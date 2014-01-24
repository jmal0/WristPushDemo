#!/usr/bin/env python

import roslib; roslib.load_manifest('wrist_demo')
import rospy
from hubomsg.msg import *
import sys
import select

class wrist_demo:

    def __init__( self ):

        rospy.init_node("wrist_push_controller")
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

        self.CALIB_LOW = 0
        self.CALIB_HIGH = 1
        self.DEMO = 2


        self.sleepVal = .01
        
        self.RSP_pos = 0
        self.REP_pos = 0
        self.RWT_val = 0
        self.last = self.OUT
        

        self.state = self.CALIB_LOW
        self.i = 0
        self.lowVal = 0.0
        self.highVal = 0.0
        self.ALPHA = .1

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
            if self.state == self.CALIB_LOW:
                if self.i > -1 and self.i < 10:
                    self.lowVal = (self.ALPHA * self.lowVal) + message.value
                    self.i += 1

            elif self.state == self.CALIB_HIGH:
                if self.i > -1 and self.i < 10:
                    self.highVal = (self.ALPHA * self.highVal) + message.value
                    self.i += 1

            elif self.state == self.DEMO:

                self.RWT_val = message.value
                
                if self.RWT_val < self.HIGH_THRESHOLD:
                    self.setIn()        
                elif self.last == self.IN and self.RWT_val > self.MID_HIGH_THRESHOLD:
                    self.setStay()
                elif self.last == self.OUT and self.RWT_val < self.MID_LOW_THRESHOLD:
                    self.setStay()
                elif self.RWT_val > self.LOW_THRESHOLD:
                    self.setOut()


def input_available():
    rlist, wlist, elist = select.select([sys.stdin], [], [], 0)
    if rlist: 
        return True
    return False

if __name__ == '__main__':
    print "Starting the wrist demo"

    shouldPrint = True

    demo = wrist_demo()
    demo.i = -1
    while not rospy.is_shutdown():
        if demo.state == demo.CALIB_LOW:
            if demo.i < 0:
                demo.setOut()
                if shouldPrint:
                    print "Press <Enter> to begin rest calibration:"
                    shouldPrint = False
                if input_available():
                    userInput = raw_input()
                    demo.i = 0
                    demo.pub.publish("","InitializeSensors","","")
                    rospy.sleep(demo.sleepVal)
            elif demo.i < 10:
                demo.pub.publish("RWT","Get","0","f_z")
                rospy.sleep(demo.sleepVal)
            elif demo.i == 10:
                demo.i = -1
                demo.state = demo.CALIB_HIGH
                shouldPrint = True
        elif demo.state == demo.CALIB_HIGH:
            if demo.i < 0:
                if shouldPrint:
                    print "Press <Enter> to begin activation calibration:"
                    shouldPrint = False
                if input_available():
                    userInput = raw_input()
                    demo.i = 0
            elif demo.i < 10:
                demo.pub.publish("RWT","Get","0","f_z")
                rospy.sleep(demo.sleepVal)

            elif demo.i == 10:
                demo.i = -1
                band = demo.highVal - demo.lowVal
                demo.LOW_THRESHOLD = demo.lowVal + (.20 * band)
                demo.MID_LOW_THRESHOLD = demo.LOW_THRESHOLD + (.25 * band)
                demo.MID_HIGH_THRESHOLD = demo.MID_LOW_THRESHOLD + (.30 * band)
                demo.HIGH_THRESHOLD = demo.MID_HIGH_THRESHOLD + (.25 * band)
                print str(demo.lowVal) + " " + str(demo.highVal)
                print str(demo.LOW_THRESHOLD) + " " + str(demo.MID_LOW_THRESHOLD) + " " + str(demo.MID_HIGH_THRESHOLD) + " " + str(demo.HIGH_THRESHOLD)

                demo.state = demo.DEMO
        else:

            demo.pub.publish("RSP","Get","0","position")
            rospy.sleep(demo.sleepVal)
            demo.pub.publish("REP","Get","0","position")
            rospy.sleep(demo.sleepVal)
            demo.pub.publish("RWT","Get","0","f_z")
            rospy.sleep(demo.sleepVal)
            
            if input_available():
                rospy.sleep(1)
                demo.pub.publish("RSP REP","position position", "0 0", "")
                rospy.sleep(demo.sleepVal)
                sys.exit(0)


