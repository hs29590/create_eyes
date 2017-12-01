#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('create_eyes')
import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

import numpy
import math


class DriveCreate2:

  def __init__(self):

    self.err_sub = rospy.Subscriber('line_error', Float32, self.errCallback)
    self.state_sub = rospy.Subscriber('create_state', String, self.stateCallback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    self.twist = Twist()
    self.state = "Drive"
    
  def stateCallback(self,stateMsg):
    if(stateMsg.data == "Drive" or stateMsg.data == "Turn" or stateMsg.data == "Stop"):
        self.state = stateMsg.data;
    else:
        print("Recd. a state: (" + stateMsg.data + ") that isn't recognized");

  def sendStopCmd(self):
    self.twist.linear.x = 0.0
    self.twist.angular.z = 0.0
    self.cmd_vel_pub.publish(self.twist)

  def errCallback(self,err):
    if(self.state == "Drive"):
        if(err.data == -1000.0):
            self.sendStopCmd();
        else:
            self.twist.linear.x = 0.2
            self.twist.angular.z = (-float(err.data) / 100) ;
            self.cmd_vel_pub.publish(self.twist)
    elif(self.state == "Stop"):
        self.sendStopCmd();

def main(args):
  rospy.init_node('drive_create2', anonymous=True)
  ic = DriveCreate2()
  rospy.spin();
        
if __name__ == '__main__':
    main(sys.argv)

