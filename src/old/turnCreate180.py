#!/usr/bin/env python
from __future__ import print_function
import tf
import time
import roslib
roslib.load_manifest('create_eyes')
import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

import numpy
import math


class DriveCreate2:

  def __init__(self):

    self.err_sub = rospy.Subscriber('line_error', Float32, self.errCallback)
    self.odom_sub = rospy.Subscriber('odom', Odometry, self.odomCallback)
    self.yaw = None;
    self.last_odom = Odometry();


    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    self.twist = Twist()

  def errCallback(self,msg):
      alpha = msg.data
      theta = self.yaw;
      rollOverReqd = False;
      if(alpha > 0):
         desired_angle = theta + alpha;
         if(desired_angle > math.pi):
            desired_angle = desired_angle - 2*math.pi;
            rollOverReqd = True;
         while(True):
            self.twist.linear.x = 0;
            self.twist.angular.z = 0.5;
            self.cmd_vel_pub.publish(self.twist)
            print("Turning: " + str(theta) + " to get to " + str(desired_angle));
            if((theta) > (desired_angle)):
                if(not rollOverReqd):
                    break;
                else:
                    if(theta < 0):
                        break;
                    
            theta = self.yaw

         self.twist.angular.z = 0;
         self.cmd_vel_pub.publish(self.twist);
      
      elif(alpha < 0):
          desired_angle = theta + alpha;
          if(desired_angle < -math.pi):
              desired_angle = desired_angle + 2*math.pi;
              rollOverReqd = True;
          while(True):
              self.twist.linear.x = 0;
              self.twist.angular.z = -0.5;
              self.cmd_vel_pub.publish(self.twist);
              print("Turning: " + str(theta) + " to get to " + str(desired_angle));
              if(theta < desired_angle):
                  if(not rollOverReqd):
                    break;
                  else:
                    if(theta > 0):
                      break;
              theta = self.yaw

          self.twist.angular.z = 0;
          self.cmd_vel_pub.publish(self.twist);

  def odomCallback(self,msg):
    self.last_odom = msg;
    quaternion = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    self.yaw = euler[2]

def main(args):
  rospy.init_node('drive_create2', anonymous=True)
  ic = DriveCreate2()
  rospy.spin();
        
if __name__ == '__main__':
    main(sys.argv)

