#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('create_eyes')
import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import tf

import numpy
import math


class DriveCreate2:

  def __init__(self):
    
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    self.mode_pub = rospy.Publisher('mode', String, queue_size = 1)

    self.err_sub = rospy.Subscriber('line_error', Float32, self.errCallback)
    self.state_sub = rospy.Subscriber('master_state', String, self.stateCallback)
    self.odom_sub = rospy.Subscriber('odom', Odometry, self.odomCallback)
    self.sonar_sub = rospy.Subscriber('sonar_drive', Bool, self.sonarCallback);

    self.twist = Twist()
    
    self.state = "Stop"
    
    self.yaw = None;
    self.last_odom = Odometry();
    self.sonar_drive = True;

  def sonarCallback(self, msg):
      self.sonar_drive = msg.data;

  def command_turn(self, angleToTurn):
      rolled_over = False;
      starting = self.yaw;
      desired = starting + angleToTurn;
      if(desired > math.pi):
          desired = desired - 2*math.pi;
      elif(desired < -math.pi):
          desired = desired + 2*math.pi;
      
      current = starting;

      if(desired > starting):
          while(True):
              self.twist.linear.x = 0;
              self.twist.angular.z = 0.5;
              self.cmd_vel_pub.publish(self.twist);
              current = self.yaw;
              if(current > desired):
                  break;
      elif(desired < starting):
          while(True):
              self.twist.linear.x = 0;
              self.twist.angular.z = -0.5;
              self.cmd_vel_pub.publish(self.twist);
              current = self.yaw;
              if(current < desired):
                  break;

      self.twist.angular.z = 0;
      self.cmd_vel_pub.publish(self.twist);
          

  def command_turn1(self,angleToTurn):
      alpha = angleToTurn
      rolled_over = False;

      theta = self.yaw;
      rollOverReqd = False;
      if(alpha > 0):
         desired_angle = theta + alpha;
         if(desired_angle > math.pi):
            desired_angle = desired_angle - 2*math.pi;
            rollOverReqd = True;
         while(True):
            rolled_over = False;
            if(theta*self.yaw < 0):
                rolled_over = True;
            self.twist.linear.x = 0;
            self.twist.angular.z = 0.5;
            self.cmd_vel_pub.publish(self.twist)
            print("Turning: " + str(theta) + " to get to " + str(desired_angle));
            if((theta) > (desired_angle)):
                if(not rollOverReqd):
                    break;
                else:
                    if(rolled_over):
                        break;
                    
            theta = self.yaw

      elif(alpha < 0):
          desired_angle = theta + alpha;
          if(desired_angle < -math.pi):
              desired_angle = desired_angle + 2*math.pi;
              rollOverReqd = True;
          while(True):
              rolled_over = False;
              if(theta * self.yaw < 0):
                  rolled_over = True;
              self.twist.linear.x = 0;
              self.twist.angular.z = -0.5;
              self.cmd_vel_pub.publish(self.twist);
              print("Turning: " + str(theta) + " to get to " + str(desired_angle));
              if(theta < desired_angle):
                  if(not rollOverReqd):
                    break;
                  else:
                    if(rolled_over):
                      break;
              theta = self.yaw
          
      self.twist.angular.z = 0;
      self.cmd_vel_pub.publish(self.twist);


  def stateCallback(self,stateMsg):
    if(stateMsg.data == "CONTROLLER_FollowLine" ):
        self.mode_pub.publish("safe");
        self.state = "FollowLine";

    elif(stateMsg.data == "CONTROLLER_Stop"):
        self.state = "Stop";
        self.mode_pub.publish("stop");
        self.sendStopCmd();
        
    elif(stateMsg.data == "CONTROLLER_Turn"):
        self.state = "Turn";
        self.mode_pub.publish("safe");
        self.command_turn(math.pi);
        self.state = "Stop";
        
    elif(stateMsg.data == "CONTROLLER_Dock"):
        self.state = "Dock";
        self.mode_pub.publish("dock");
        
    elif(stateMsg.data == "CONTROLLER_UnDock"):
        self.state = "UnDock";
        self.mode_pub.publish("safe");
        self.undock();
        self.state = "Stop";

    else:
        print("Recd. a state: (" + stateMsg.data + ") that isn't recognized");

    print("Current state is: " + self.state);

  def undock(self):
      self.twist.linear.x = -0.5;
      self.twist.angular.z = 0.0;
      self.cmd_vel_pub.publish(self.twist);
      time.sleep(4);
      self.sendStopCmd();
      self.command_turn(math.pi);
      self.sendStopCmd();

  def sendStopCmd(self):
    self.twist.linear.x = 0.0
    self.twist.angular.z = 0.0
    self.cmd_vel_pub.publish(self.twist)
    
  def odomCallback(self,msg):
    self.last_odom = msg;
    quaternion = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    self.yaw = euler[2]

  def errCallback(self,err):
    if(self.state == "FollowLine" and self.sonar_drive):
        if(err.data == -1000.0):
            pass
        else:
            self.twist.linear.x = 0.2
            self.twist.angular.z = (-float(err.data) / 100) ;
            self.cmd_vel_pub.publish(self.twist)

def main(args):
  rospy.init_node('create_eyes_controller', anonymous=True)
  ic = DriveCreate2()
  rospy.spin();
        
if __name__ == '__main__':
    main(sys.argv)
