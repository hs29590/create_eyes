import RPi.GPIO as GPIO                    #Import GPIO library

import time                                #Import time library

global Ultrasoundswitch			   #global declaration

GPIO.setmode(GPIO.BCM)                     #Set GPIO pin numbering 



TRIG = 20 #Associate pin 23 to TRIG

ECHO = 26                              #Associate pin 24 to ECHO


def setup():
    GPIO.setup(TRIG,GPIO.OUT)                  #Set pin as GPIO out

    GPIO.setup(ECHO,GPIO.IN)                   #Set pin as GPIO in



def loop():

       time.sleep(0.0002)                            #Delay of 2 seconds(just to be safe)

       GPIO.output(TRIG, True)                  #Set TRIG as HIGH

       time.sleep(0.00001)                      #Delay of 0.00001 seconds

       GPIO.output(TRIG, False)                 #Set TRIG as LOW

       while GPIO.input(ECHO)==0:               #Check whether the ECHO is LOW

             pulse_start = time.time()              #Saves the last known time of LOW pulse (it will note the time until the echo goes high the moment echo goes high the last time stamp is recoded)



       while GPIO.input(ECHO)==1:               #Check whether the ECHO is HIGH

             pulse_end = time.time()                #Saves the last known time of HIGH pulse(it will note the time until the echo goes low the moment echo goes low the last time stamp is recoded) 



       pulse_duration = pulse_end - pulse_start #Get pulse duration to a variable (difference between 2 times)



       distance = pulse_duration * 17150        #Multiply pulse duration by 17150 to get distance

       distance = round(distance, 2)            #Round to two decimal points

       print("Distance: " + str(distance));

setup()

while True: 
      loop()
      time.sleep(0.5);
