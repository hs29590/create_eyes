#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('create_eyes')
from std_msgs.msg import String
from std_msgs.msg import Bool
import sys
import rospy
import time
import RPi.GPIO as GPIO   #import the GPIO library
import time               #import the time library

class Buzzer(object):
 def __init__(self):
  GPIO.setmode(GPIO.BCM)  
  self.buzzer_pin = 17 #set to GPIO pin 5
  GPIO.setup(self.buzzer_pin, GPIO.IN)
  GPIO.setup(self.buzzer_pin, GPIO.OUT)
  print("buzzer ready")

 def __del__(self):
  class_name = self.__class__.__name__
  print (class_name, "finished")

 def buzz(self,pitch, duration):   #create the function “buzz” and feed it the pitch and duration)
 
  if(pitch==0):
   time.sleep(duration)
   return
  period = 1.0 / pitch     #in physics, the period (sec/cyc) is the inverse of the frequency (cyc/sec)
  delay = period / 2     #calcuate the time for half of the wave  
  cycles = int(duration * pitch)   #the number of waves to produce is the duration times the frequency

  for i in range(cycles):    #start a loop from 0 to the variable “cycles” calculated above
   GPIO.output(self.buzzer_pin, True)   #set pin 18 to high
   time.sleep(delay)    #wait with pin 18 high
   GPIO.output(self.buzzer_pin, False)    #set pin 18 to low
   time.sleep(delay)    #wait with pin 18 low

 def play(self, tune):
  GPIO.setmode(GPIO.BCM)
  GPIO.setup(self.buzzer_pin, GPIO.OUT)
  x=0

  print("Playing tune ",tune)
  if(tune==1):
    pitches=[262,294,330,349,392,440,494,523, 587, 659,698,784,880,988,1047]
    duration=0.1
    for p in pitches:
      self.buzz(p, duration)  #feed the pitch and duration to the function, “buzz”
      time.sleep(duration *0.5)
    for p in reversed(pitches):
      self.buzz(p, duration)
      time.sleep(duration *0.5)

  elif(tune==2):
    pitches=[262,330,392,523,1047]
    duration=[0.2,0.2,0.2,0.2,0.2,0,5]
    for p in pitches:
      self.buzz(p, duration[x])  #feed the pitch and duration to the function, “buzz”
      time.sleep(duration[x] *0.5)
      x+=1
  elif(tune==3):
    pitches=[392,294,0,392,294,0,392,0,392,392,392,0,1047,262]
    duration=[0.2,0.2,0.2,0.2,0.2,0.2,0.1,0.1,0.1,0.1,0.1,0.1,0.8,0.4]
    for p in pitches:
      self.buzz(p, duration[x])  #feed the pitch and duration to the func$
      time.sleep(duration[x] *0.5)
      x+=1

  elif(tune==4):
    pitches=[1047, 988,659]
    duration=[0.1,0.1,0.2]
    for p in pitches:
      self.buzz(p, duration[x])  #feed the pitch and duration to the func$
      time.sleep(duration[x] *0.5)
      x+=1

  elif(tune==5):
    pitches=[1047, 988,523]
    duration=[0.1,0.1,0.2]
    for p in pitches:
      self.buzz(p, duration[x])  #feed the pitch and duration to the func$
      time.sleep(duration[x] *0.5)
      x+=1

  GPIO.setup(self.buzzer_pin, GPIO.IN)

if __name__ == "__main__":
  rospy.init_node('beep_node_create', anonymous=True)
  a = input("Enter Tune number 1-5:")
  buzzer = Buzzer()
  buzzer.play(int(a))
ss='''
class SonarClass:
   
    def cbfunc1(self,gpio, level, tick):
       if level == 0: # echo line changed from high to low.
          if self.high_tick1 is not None:
             echo = pigpio.tickDiff(self.high_tick1, tick)
             cms = (echo / 1000000.0) * 34030 / 2
             #print("echo1 was {} micros long ({:.1f} cms)".format(echo, cms))
             self.echo1Distance = self.echo1Distance*0.5 + 0.5*cms;
       else:
          self.high_tick1 = tick

    def cbfunc2(self,gpio, level, tick):
       if level == 0: # echo line changed from high to low.
          if self.high_tick2 is not None:
             echo = pigpio.tickDiff(self.high_tick2, tick)
             cms = (echo / 1000000.0) * 34030 / 2
             #print("echo2 was {} micros long ({:.1f} cms)".format(echo, cms))
             self.echo2Distance = self.echo2Distance*0.5 + 0.5*cms;
       else:
          self.high_tick2 = tick

    def cbfunc3(self,gpio, level, tick):
       if level == 0: # echo line changed from high to low.
          if self.high_tick3 is not None:
             echo = pigpio.tickDiff(self.high_tick3, tick)
             cms = (echo / 1000000.0) * 34030 / 2
             #print("echo3 was {} micros long ({:.1f} cms)".format(echo, cms))
             self.echo3Distance = self.echo3Distance*0.5 + 0.5*cms;
       else:
          self.high_tick3 = tick
          
    def __init__ (self):

        self.TRIGGER=19
        self.ECHO1=21
        self.ECHO2=26
        self.ECHO3=20
   
        self.echo1Distance = 1000;
        self.echo2Distance = 1000;
        self.echo3Distance = 1000;

        self.USSensorStoppingRange = 20.0;

        self.high_tick1 = None # global to hold high tick.
        self.high_tick2 = None # global to hold high tick.
        self.high_tick3 = None # global to hold high tick.
    
        self.state_pub = rospy.Publisher('sonar_drive', Bool, queue_size=1)
        self.safeToDrive = True;

        self.pi = pigpio.pi() # Connect to local Pi.

        self.pi.set_mode(self.TRIGGER, pigpio.OUTPUT)
        self.pi.set_mode(self.ECHO1, pigpio.INPUT)
        self.pi.set_mode(self.ECHO2, pigpio.INPUT)
        self.pi.set_mode(self.ECHO3, pigpio.INPUT)

        self.cb1 = self.pi.callback(self.ECHO1, pigpio.EITHER_EDGE, self.cbfunc1)
        self.cb2 = self.pi.callback(self.ECHO2, pigpio.EITHER_EDGE, self.cbfunc2)
        self.cb3 = self.pi.callback(self.ECHO3, pigpio.EITHER_EDGE, self.cbfunc3)
   
    def __del__(self):
        self.cb1.cancel() # Cancel callback.
        self.cb2.cancel() # Cancel callback.
        self.cb3.cancel() # Cancel callback.
        self.pi.stop() # Close connection to Pi 

    def triggerAndPublish(self):
        self.safeToDrive = True;
        self.pi.gpio_trigger(self.TRIGGER, 10);
        time.sleep(0.1);
        if(self.echo1Distance < self.USSensorStoppingRange or self.echo2Distance < self.USSensorStoppingRange or self.echo3Distance < self.USSensorStoppingRange):
            self.state_pub.publish(False);
        else:
            self.state_pub.publish(True);


def main(args):
  rospy.init_node('sonar_node_create', anonymous=True)
  sc = SonarClass()
#  rospy.spin();
  while not rospy.is_shutdown():
    sc.triggerAndPublish(); #this sleeps for 0.2s internally
        
if __name__ == '__main__':
    main(sys.argv)'''

