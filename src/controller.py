#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('create_eyes')
import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import tf
import time
import numpy
import math


class DriveCreate2:

  def __init__(self):
    
    self.state = "Stop"
    self.twist = Twist()
    self.yaw = None;
    self.last_odom = Odometry();
    self.sonar_drive = True;

    self.LINEAR_SPEED = 0.4;
        
    self.odomRecd = False;

    self.cmd_vel_pub = rospy.Publisher('iRobot_0/cmd_vel', Twist, queue_size=1)
    self.mode_pub = rospy.Publisher('iRobot_0/mode', String, queue_size = 1)

    self.err_sub = rospy.Subscriber('line_error', Float32, self.errCallback)
    self.state_sub = rospy.Subscriber('master_state', String, self.stateCallback)
    self.odom_sub = rospy.Subscriber('iRobot_0/odom', Odometry, self.odomCallback)
    self.sonar_sub = rospy.Subscriber('sonar_drive', Bool, self.sonarCallback);
#    self.pid_err_sub = rospy.Subscriber('control_effort', Float64, self.ctrleffortCallback)

    self.isStopped = True;

    self.last_drive_lin = 0.0;
    self.last_drive_ang = 0.0;

    self.noLineCount = 0;

  def smooth_drive(self,lin, ang):
      self.twist.linear.x = self.last_drive_lin*0.5 + lin*0.5;
      #self.twist.angular.z = self.last_drive_ang*0.5 + ang*0.5;

      #self.twist.linear.x = lin;
      self.twist.angular.z = ang;

      if(self.twist.linear.x < 0.05):
          self.twist.linear.x = 0.0;
          #self.twist.angular.z = 0.0;
          
      self.cmd_vel_pub.publish(self.twist);

 #     self.last_drive_lin = self.twist.linear.x;
      #self.last_drive_ang = self.twist.angular.z;

  def sonarCallback(self, msg):
      self.sonar_drive = msg.data;

  def command_turn(self, angleToTurn):
      if(not self.odomRecd):
          rospy.loginfo("Trying to turn without odom recd.");
          return;
      starting = self.yaw;
      desired = starting + angleToTurn;
      if(desired > math.pi):
          desired = desired - 2*math.pi;
      elif(desired < -math.pi):
          desired = desired + 2*math.pi;
      
      current = starting;

      if(desired > starting):
          t_end = time.time() + 30;
          while(True):
              self.smooth_drive(0.0,0.5);
 #             self.twist.linear.x = 0;
 #             self.twist.angular.z = 0.5;
 #             self.cmd_vel_pub.publish(self.twist);
              time.sleep(0.01);
              current = self.yaw;
              #rospy.loginfo_throttle(60, "This message will print every 60 seconds")
              rospy.loginfo_throttle(5,"Turning: " + str(current) + " " + str(desired));
              if(current > desired or current < starting):
                  break;
              if(time.time() > t_end):
                  rospy.logwarn("Stopping the turn, couldn't finish it");
                  break;
      elif(desired < starting):
          t_end = time.time() + 30;
          while(True):
              self.smooth_drive(0.0,-0.5);
#              self.twist.linear.x = 0;
#              self.twist.angular.z = -0.5;
#              self.cmd_vel_pub.publish(self.twist);
              time.sleep(0.001);
              current = self.yaw;
              #rospy.loginfo("Turning: " + str(current) + " " + str(desired));
              rospy.loginfo_throttle(5,"Turning: " + str(current) + " " + str(desired));
              if(current < desired or current > starting):
                  break;
              if(time.time() > t_end):
                  rospy.logwarn("Stopping the turn, couldn't finish it");
                  break;

      self.sendStopCmd();
          
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
        self.mode_pub.publish("clean");
        time.sleep(8.5);
        self.mode_pub.publish("safe");

        self.state = "Stop";
        self.sendStopCmd();

    elif(stateMsg.data == "CONTROLLER_Reset"):
        rospy.loginfo("Publishing reset, waiting 7 sec..");
        self.mode_pub.publish("reset");
        time.sleep(7);
        self.mode_pub.publish("start");
        rospy.loginfo("Published Start..");

    else:
        rospy.logwarn("Recd. a state: (" + stateMsg.data + ") that isn't recognized");

    rospy.loginfo("Current state is: " + self.state);

  
  def sendStopCmd(self):
      self.smooth_drive(0.0,0.0);
#    lin_v = self.LINEAR_SPEED - 0.05;
#    self.twist.angular.z = 0;
#    while(lin_v > 0):
#        self.twist.linear.x = lin_v;
#        lin_v = lin_v - 0.05;
#        #self.cmd_vel_pub.publish(self.twist);
#
#    self.isStopped = True;
#    self.twist.linear.x = 0.0
#    self.twist.angular.z = 0.0
#    self.cmd_vel_pub.publish(self.twist)
    
  def odomCallback(self,msg):
    if(not self.odomRecd):
        self.odomRecd = True;
    self.last_odom = msg;
    quaternion = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    self.yaw = euler[2]

  def errCallback(self,err):
    if(self.state == "FollowLine" and self.sonar_drive):
        if(err.data == -1000.0):
            #I will come here when I'm asked to follow line, but I can't see the line. User is expected to press go button again.
            #this is also done to stop the robot from following random things if it doesn't see the line
            self.noLineCount = self.noLineCount + 1;
            if(self.noLineCount >= 20):
                rospy.loginfo_throttle(5,"Stopping since line isn't visible");
                self.smooth_drive(0.0,0.0);
                self.state = "Stop";
        else:
            self.smooth_drive(self.LINEAR_SPEED, (-float(err.data)/50.0));
#            rospy.logwarn_throttle(10,"here drive is commented out...");
            self.noLineCount = 0;

#  def ctrleffortCallback(self,err):
  #    if(self.state == "FollowLine" and self.sonar_drive):
#          self.smooth_drive(self.LINEAR_SPEED, (-float(err.data)/50.0));

#                self.isStopped = False;
#                lin_v = 0;
#                while(lin_v < self.LINEAR_SPEED):
#                    lin_v = lin_v + 0.1;
#                    self.twist.linear.x = lin_v;
#                    self.twist.angular.z = (-float(err.data) / 100);
#                    self.cmd_vel_pub.publish(self.twist);
#            else:
#                self.isStopped = False;
#                self.twist.linear.x = self.LINEAR_SPEED;
#                self.twist.angular.z = (-float(err.data) / 50) ;
#                self.cmd_vel_pub.publish(self.twist);
#                self.noLineCount = 0;

def main(args):
  rospy.init_node('create_eyes_controller', anonymous=True)
  ic = DriveCreate2()
  rospy.spin();
        
if __name__ == '__main__':
    main(sys.argv)

