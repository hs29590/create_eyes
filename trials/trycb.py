#!/usr/bin/env python

# 2014-08-18 hc-sr04.py

import time

import pigpio

TRIGGER=19
ECHO1=21
ECHO2=26
ECHO3=20

high_tick1 = None # global to hold high tick.
high_tick2 = None # global to hold high tick.
high_tick3 = None # global to hold high tick.

def cbfunc1(gpio, level, tick):
   global high_tick1
   if level == 0: # echo line changed from high to low.
      if high_tick1 is not None:
         echo = pigpio.tickDiff(high_tick1, tick)
         cms = (echo / 1000000.0) * 34030 / 2
         print("echo1 was {} micros long ({:.1f} cms)".format(echo, cms))
   else:
      high_tick1 = tick

def cbfunc2(gpio, level, tick):
   global high_tick2
   if level == 0: # echo line changed from high to low.
      if high_tick2 is not None:
         echo = pigpio.tickDiff(high_tick2, tick)
         cms = (echo / 1000000.0) * 34030 / 2
         print("echo2 was {} micros long ({:.1f} cms)".format(echo, cms))
   else:
      high_tick2 = tick

def cbfunc3(gpio, level, tick):
   global high_tick3
   if level == 0: # echo line changed from high to low.
      if high_tick3 is not None:
         echo = pigpio.tickDiff(high_tick3, tick)
         cms = (echo / 1000000.0) * 34030 / 2
         print("echo3 was {} micros long ({:.1f} cms)".format(echo, cms))
   else:
      high_tick3 = tick

pi = pigpio.pi() # Connect to local Pi.

pi.set_mode(TRIGGER, pigpio.OUTPUT)
pi.set_mode(ECHO1, pigpio.INPUT)
pi.set_mode(ECHO2, pigpio.INPUT)
pi.set_mode(ECHO3, pigpio.INPUT)

cb1 = pi.callback(ECHO1, pigpio.EITHER_EDGE, cbfunc1)
cb2 = pi.callback(ECHO2, pigpio.EITHER_EDGE, cbfunc2)
cb3 = pi.callback(ECHO3, pigpio.EITHER_EDGE, cbfunc3)

start = time.time()

while (time.time()-start) < 60:
   pi.gpio_trigger(TRIGGER, 10)
   time.sleep(1)

cb1.cancel() # Cancel callback.
cb2.cancel() # Cancel callback.
cb3.cancel() # Cancel callback.
pi.stop() # Close connection to Pi 
