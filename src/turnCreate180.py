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
 
    self.TimeToHalfAfterNoErrRecd = rospy.Duration(1); #Halt if we don't recv. a cmd_msg for these many seconds
 
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    self.twist = Twist()
    self.lastErrorMsgRecdTime = rospy.Time.now() - self.TimeToHalfAfterNoErrRecd;

  def errCallback(self,err):
    if(err.data == -1000.0):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
    else:
        self.lastErrorMsgRecdTime = rospy.Time.now();
        self.twist.linear.x = 0.2
        self.twist.angular.z = -float(err.data) / 100
        self.cmd_vel_pub.publish(self.twist)

def main(args):
  rospy.init_node('drive_create2', anonymous=True)
  ic = DriveCreate2()
  rospy.spin();
        
if __name__ == '__main__':
    main(sys.argv)

