#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('create_eyes')
import sys
import rospy
import time
#import _thread
import numpy
import math

#from tkinter import *
#from tkinter import ttk
from Tkinter import *
import ttk


from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from irobotcreate2.msg import Battery


class Create2StatePublisher:

  def __init__(self):

    #self.gui_sub = rospy.Subscriber('gui_state', String, self.guiCallback)
    self.bat_sub = rospy.Subscriber('iRobot_0/battery', Battery, self.batteryCallback)

    self.line_visible_sub = rospy.Subscriber('line_visible', Bool, self.lineVisibleCallback)

    self.master_state_pub = rospy.Publisher('master_state', String, queue_size=1)

    self.gui_state = "Stop";
    
    self.last_gui_state = "Stop";
    
    
    #expect to start at Dock
    self.heading = "B";

    self.docked = True;

    self.publish("CONTROLLER_Stop");
    
    self.status = "Started"
    

    ##FROM GUI###
    self.battery = "N/A"
    self.facingTowardsA = False;
    #self.last_state_published = "Stop";
    #self.current_state = "Stop";

    self.root = Tk()
    self.root.title("GUI")
    
    framew = 500; # root w
    frameh = 400; # root h
    screenw = self.root.winfo_screenwidth();
    screenh = self.root.winfo_screenheight();
    posx = (screenw/2) - (framew/2);
    posy = (screenh/2) - (frameh/2);
    self.root.geometry( "%dx%d+%d+%d" % (framew,frameh,posx,posy))

    self.currentStatus = StringVar();
    self.currentStatus.set('Stop');

    self.batteryStatus = StringVar();
    self.batteryStatus.set('na%');
    
    self.headingStatus = StringVar();
    self.headingStatus.set('NA');
    
    self.lineVisible = StringVar();
    self.lineVisible.set(str(False));

    self.mainframe = ttk.Frame(self.root, padding="10 10 30 30", height=400, width=500)
    self.mainframe.grid(column=0, row=0, sticky=(N, W, E, S))
    self.mainframe.columnconfigure(0, weight=50,minsize=50)
    self.mainframe.rowconfigure(0, weight=50,minsize=50)

    self.buttonStyle = ttk.Style()
    self.buttonStyle.configure('my.TButton', font=('Helvetica', 18))
    
    self.batteryLabel = ttk.Label(self.mainframe, textvariable=self.batteryStatus, font=('Helvetica',12));
    self.batteryLabel.grid(row=1, column=1);

    self.statusLabel = ttk.Label(self.mainframe, textvariable=self.currentStatus, font=('Helvetica',12));
    self.statusLabel.grid(row=1, column=0);

    self.headLabel = ttk.Label(self.mainframe, textvariable=self.headingStatus, font=('Helvetica',12));
    self.headLabel.grid(row=0,column=1);

    self.lineLabel = ttk.Label(self.mainframe, textvariable=self.lineVisible, font=('Helvetica',12));
    self.lineLabel.grid(row=0, column=0);

    ttk.Button(self.mainframe, text="Go To A", style='my.TButton', command=self.goToA, width=16).grid(row=2, rowspan=2, column=0, pady=25)
    ttk.Button(self.mainframe, text="Go To B/DOCK", style='my.TButton', command=self.goToB, width=16).grid(row=2, rowspan=2, column=1, pady=25)
    ttk.Button(self.mainframe, text="STOP", style='my.TButton', command=self.Stop, width=16).grid(row=4, rowspan=3, column=0, columnspan=3, pady=5)
    
    self.root.after(1000, self.updateLabel);
    
    ##END FROM GUI##
 
  def goToA(self):
      #print("if not facing towards A ( turn 180), start following line, publish, turn, or drive state");
      rospy.loginfo("GotoA");
      if(self.last_gui_state != "GoToA"):
#      self.last_gui_state = "GoToA";
          self.guiButtonUpdate("GoToA");
  def goToB(self):
      rospy.loginfo("GoToB");
      #print("if facing towards A (turn 180), start following Line, publish, turn or follow line state");
      if(self.last_gui_state != "GoToB"):
#         self.last_gui_state = "GoToB";
          self.guiButtonUpdate("GoToB");

  def Stop(self):
      rospy.loginfo("Stop");
      if(self.last_gui_state != "Stop"):
#         self.last_gui_state = "Stop";
          self.guiButtonUpdate("Stop");

  def updateLabel(self):
#      if(self.current_state != self.last_state_published):
#          call(["rostopic", "pub", "-1", "/gui_state", "std_msgs/String", self.current_state])
#          self.last_state_published = self.current_state;
      self.currentStatus.set(self.last_gui_state);
      self.headingStatus.set(self.heading);
      self.statusLabel.update_idletasks();
      self.batteryLabel.update_idletasks();
      self.headLabel.update_idletasks();
      self.lineLabel.update_idletasks();

      self.root.update_idletasks();
      self.root.after(200, self.updateLabel);
 
  def lineVisibleCallback(self,msg):
      self.lineVisible.set("Line: " + str(msg.data));

  def batteryCallback(self,msg):
      self.docked = msg.dock;
      self.batteryStatus.set(str("%.2f" % round(msg.level,2))+"%, Docked: " + str(self.docked));
       
  def publish(self,msg):
      self.master_state_pub.publish(msg);
      self.last_state_published = msg;
      rospy.loginfo("Published: " + msg);

  def turnAround(self):
      self.publish("CONTROLLER_Turn");
      if(self.heading == "A"):
          self.heading = "B";
      elif(self.heading == "B"):
          self.heading = "A";

  def guiButtonUpdate(self,msg):
      if(msg == "GoToA"):
#          if(self.docked):
#              self.publish("CONTROLLER_UnDock");
#              time.sleep(5);
#              self.headng = "A";
#          
          if(self.last_gui_state == "Stop"):
              if(self.heading == "A"):
                  self.publish("CONTROLLER_FollowLine");
              elif(self.heading == "B"):
                  self.turnAround();
                  self.publish("CONTROLLER_FollowLine");

      elif(msg == "GoToB"):
          if(self.last_gui_state == "Stop"):
              if(self.heading == "B"):
                  self.publish("CONTROLLER_FollowLine");
#                  self.publish("CONTROLLER_Dock");
#                  time.sleep(5);
#                  if(not self.docked):
#                      self.publish("CONTROLLER_FollowLine");
              elif(self.heading == "A"):
                  self.turnAround();
                  self.publish("CONTROLLER_FollowLine");

      elif(msg == "Stop"):
          self.publish("CONTROLLER_Stop");

      else:
          rospy.logwarn("Unknown state: " + msg);
    
      self.last_gui_state = msg;

def main(args):
  rospy.init_node('create_eyes_state_publisher', anonymous=True)
  ic = Create2StatePublisher()
  ic.root.mainloop();
#  rospy.spin();
        
if __name__ == '__main__':
    main(sys.argv)

