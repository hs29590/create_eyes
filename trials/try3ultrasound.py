import RPI.GPIO as GPIO
import time
global pulse1_start
global pulse2_start
global pulse3_start
global pulse1_end
global pulse2_end
global pulse3_end

GPIO.setmode(GPIO.BCM)


Trig  = 20 #Associate pin 23 to TRIG
Echo1 = 21 #Associate pin 24 to ECHO
Echo2 = 26 #pin 37 as led
Echo3 = 19 #pin 35 as button 

def setup():
    GPIO.setup(Trig,GPIO.OUT)
    GPIO.setup(Echo1,GPIO.IN)
    GPIO.setup(Echo2,GPIO.IN)
    GPIO.setup(Echo3,GPIO.IN)
    GPIO.setup(Led,GPIO,IN)
    GPIO.setup(Button,GPIO.IN,pull_up_down=GPIO.PUD_UP)

def echo1LTH(channel):
    global pulse1_start
    pulse1_start= time.time()

def echo2LTH(channel):
    global pulse2_start
    pulse2_start=time.time()

def echo3LTH(channel):
    global pulse3_start
    pulse3_start=time.time()

def echo1HTL(channel):
    global pulse1_end;
    pulse1_end=time.time()     

def echo2HTL(channel):
    global pulse2_end;
    pulse2_end=time.time()

def echo3HTL(channel):
    global pulse3_end
    pulse3_end=time.time()    

GPIO.add_event_detect(Trig, GPIO.RISING, callback=echo1LTH) 
GPIO.add_event_detect(Trig, GPIO.RISING, callback=echo2LTH)
GPIO.add_event_detect(Trig, GPIO.RISING, callback=echo3LTH)
GPIO.add_event_detect(Echo1, GPIO.FALLING, callback=echo1HTL)
GPIO.add_event_detect(Echo2, GPIO.FALLING, callback=echo2HTL)
GPIO.add_event_detect(Echo3, GPIO.FALLING, callback=echo3HTL)

setup()

while True:
    
    global pulse1_start
    global pulse2_start
    global pulse3_start
    global pulse1_end
    global pulse2_end
    global pulse3_end

    GPIO.output(Trig, True)                  #Set TRIG as HIGH
    time.sleep(0.00001)                      #Delay of 0.00001 seconds
    GPIO.output(Trig, False) 
    time.sleep(0.3); #delay between 2 triggers
      
    pulse_duration1 = pulse1_end - pulse1_start #Get pulse duration to a variable (difference between 2 times)
    distance1 = pulse1_duration * 17150        #Multiply pulse duration by 17150 to get distance
    distance1 = round(distance1, 2)            #Round to two decimal points

    pulse_duration2 = pulse2_end - pulse2_start #Get pulse duration to a variable (difference between 2 times)
    distance2 = pulse2_duration * 17150        #Multiply pulse duration by 17150 to get distance
    distance2 = round(distance2, 2)            #Round to two decimal points

    pulse_duration3 = pulse3_end - pulse3_start #Get pulse duration to a variable (difference between 2 times)
    distance3 = pulse3_duration * 17150        #Multiply pulse duration by 17150 to get distance
    distance3 = round(distance3, 2)            #Round to two decimal points

    print("Distance1: " + str(distance1));
    print("Distance2: " + str(distance2));
    print("Distance3: " + str(distance3));
    print("---");
