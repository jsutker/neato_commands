#!/usr/bin/env python
"""Robot Controller which responds to speech inputs"""

import rospy
from sensor_msgs.msg import CompressedImage
from neato_node.msg import Bump
from geometry_msgs.msg import TwistWithCovariance,Twist,Vector3
from std_msgs.msg import String
import time
import tty
import select
import sys
import termios
import thread
from scipy import misc
from scipy import special



class Control_Robot():

    def __init__(self):
        """ Initialize the robot control, """
        rospy.init_node('robot_controller')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sleepy = rospy.Rate(2)
        #subscribe to state commands
        rospy.Subscriber('/speech_cmd', String, self.getspeech)
        #Stop the robot on shutdown
        rospy.on_shutdown(self.stop)
        #Start thread with just getkey
        thread.start_new_thread(self.getKey,())
        # make dictionary that calls functions for teleop
        self.state = {'i':self.forward, ',':self.backward,
                      'l':self.rightTurn, 'j':self.leftTurn,
                      'k':self.stop}
        #Acceptable keys that won't just stop the robot
        self.acceptablekeys = ['i','l','k',',','j']
        #Stops the robot on init just in case 
        self.linearVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.angularVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.sendMessage()
        # get key interupt things
        self.settings = termios.tcgetattr(sys.stdin)
        self.key = None


    def getKey(self):
        """ Interrupt that gets a non interrupting keypress """
        while not rospy.is_shutdown():
            tty.setraw(sys.stdin.fileno())
            select.select([sys.stdin], [], [], 0)
            self.key = sys.stdin.read(1)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            time.sleep(.05)

    def getspeech(self,msg):
        """ sets value of msg to be key presssed, controlling robot based on speech"""
        self.key = msg
 

    ##control functions

    def forward(self):
        """
            Sets the velocity to forward on keypress
        """
        #print('forward\r')
        self.linearVector  = Vector3(x=1.0, y=0.0, z=0.0)
        self.angularVector = Vector3(x=0.0, y=0.0, z=0.0)

    def backward(self):
        #print('backward\r')
        """ Sets the velocity to backward """
        self.linearVector  = Vector3(x=-1.0, y=0.0, z=0.0)
        self.angularVector = Vector3(x=0.0, y=0.0, z=0.0)

    def leftTurn(self):
        #print('leftTurn\r')
        """ Sets the velocity to turn left """
        self.linearVector  = Vector3(x=0.0, y=0.0, z=0.0)
        self.angularVector = Vector3(x=0.0, y=0.0, z=1.0)

    def rightTurn(self):
        #print('rightTurn\r')
        """ Sets the velocity to turn right """

        self.linearVector  = Vector3(x=0.0, y=0.0, z=0.0)
        self.angularVector = Vector3(x=0.0, y=0.0, z=-1.0)

    def stop(self):
        """ Sets the velocity to stop """
        #print('stop\r')
        self.linearVector  = Vector3(x=0.0, y=0.0, z=0.0)
        self.angularVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.sendMessage()

    def sendMessage(self):
        """ Publishes the Twist containing the linear and angular vector """
        #print('sendMessage\r')
        self.pub.publish(Twist(linear=self.linearVector, angular=self.angularVector))



    ##Main

    def run(self):

        while self.key != '\x03' and not rospy.is_shutdown():
            if self.key in self.acceptablekeys:
                #if an acceptable keypress, do the action
                self.state[self.key].__call__()
            else:
                # on any other keypress, stop the robot
                self.state['k'].__call__()
            self.sendMessage()
        self.sleepy.sleep()

control = Control_Robot()
control.run()
