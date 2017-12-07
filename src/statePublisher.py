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

import numpy
import math


class Create2StatePublisher:

  def __init__(self):

    self.gui_sub = rospy.Subscriber('gui_state', String, self.guiCallback)

    self.master_state_pub = rospy.Publisher('master_state', String, queue_size=1)

    self.gui_state = "Stop";
    
    self.last_gui_state = "Stop";
    
    #it is at Dock, so Undock    
    self.publish("CONTROLLER_UnDock");

    time.sleep(5);

    self.heading = "A";

    self.publish("CONTROLLER_Stop");
  
  def publish(self,msg):
      self.master_state_pub.publish(msg);
      self.last_state_published = msg;

  def turnAround(self):
      self.publish("CONTROLLER_Turn");
      if(self.heading == "A"):
          self.heading = "B";
      elif(self.heading == "B"):
          self.heading = "A";

  def guiCallback(self,msg):
      if(msg.data == "GoToA"):
          if(self.last_gui_state == "Stop"):
              if(self.heading == "A"):
                  self.publish("CONTROLLER_FollowLine");
              elif(self.heading == "B"):
                  self.turnAround();
                  self.publish("CONTROLLER_FollowLine");

      elif(msg.data == "GoToB"):
          if(self.last_gui_state == "Stop"):
              if(self.heading == "B"):
                  #self.publish("CONTROLLER_FollowLine");
                  self.publish("CONTROLLER_Dock");
              elif(self.heading == "A"):
                  self.turnAround();
                  self.publish("CONTROLLER_FollowLine");

      elif(msg.data == "Stop"):
          self.publish("CONTROLLER_Stop");

      else:
          print("Unknown state: " + msg.data);
    
      self.last_gui_state = msg.data;

def main(args):
  rospy.init_node('create_eyes_state_publisher', anonymous=True)
  ic = Create2StatePublisher()
  rospy.spin();
        
if __name__ == '__main__':
    main(sys.argv)

