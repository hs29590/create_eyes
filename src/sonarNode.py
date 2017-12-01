#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('create_eyes')
import sys
import rospy
import time
import pigpio


class SonarClass:

    self.TRIGGER=19
    self.ECHO1=21
    self.ECHO2=26
    self.ECHO3=20

    self.high_tick1 = None # global to hold high tick.
    self.high_tick2 = None # global to hold high tick.
    self.high_tick3 = None # global to hold high tick.

    self.state_pub = rospy.Publisher('create_state', String, queue_size=1)
    self.desired_state = "Drive";
   
    def cbfunc1(gpio, level, tick):
       if level == 0: # echo line changed from high to low.
          if self.high_tick1 is not None:
             echo = pigpio.tickDiff(self.high_tick1, tick)
             cms = (echo / 1000000.0) * 34030 / 2
             #print("echo1 was {} micros long ({:.1f} cms)".format(echo, cms))
             if(cms < 12.0):
               self.desired_state = "Stop";
       else:
          self.high_tick1 = tick

    def cbfunc2(gpio, level, tick):
       if level == 0: # echo line changed from high to low.
          if self.high_tick2 is not None:
             echo = pigpio.tickDiff(self.high_tick2, tick)
             cms = (echo / 1000000.0) * 34030 / 2
             #print("echo2 was {} micros long ({:.1f} cms)".format(echo, cms))
             if(cms < 12.0):
               self.desired_state = "Stop";
       else:
          self.high_tick2 = tick

    def cbfunc3(gpio, level, tick):
       if level == 0: # echo line changed from high to low.
          if self.high_tick3 is not None:
             echo = pigpio.tickDiff(self.high_tick3, tick)
             cms = (echo / 1000000.0) * 34030 / 2
             #print("echo3 was {} micros long ({:.1f} cms)".format(echo, cms))
             if(cms < 12.0):
               self.desired_state = "Stop";
       else:
          self.high_tick3 = tick
          
    def __init__ (self):

        self.pi = pigpio.pi() # Connect to local Pi.

        self.pi.set_mode(self.TRIGGER, pigpio.OUTPUT)
        self.pi.set_mode(self.ECHO1, pigpio.INPUT)
        self.pi.set_mode(self.ECHO2, pigpio.INPUT)
        self.pi.set_mode(self.ECHO3, pigpio.INPUT)

        self.cb1 = pi.callback(self.ECHO1, pigpio.EITHER_EDGE, self.cbfunc1)
        self.cb2 = pi.callback(self.ECHO2, pigpio.EITHER_EDGE, self.cbfunc2)
        self.cb3 = pi.callback(self.ECHO3, pigpio.EITHER_EDGE, self.cbfunc3)
   
    def __del__(self):
        self.cb1.cancel() # Cancel callback.
        self.cb2.cancel() # Cancel callback.
        self.cb3.cancel() # Cancel callback.
        self.pi.stop() # Close connection to Pi 


def main(args):
  rospy.init_node('sonar_node_create', anonymous=True)
  sc = SonarClass()
  rospy.spin();
  while(True):
    sc.desired_state = "Drive";
    pi.gpio_trigger(TRIGGER, 10)
    time.sleep(0.2);
    self.state_pub.publish(sc.desired_state);
        
if __name__ == '__main__':
    main(sys.argv)
