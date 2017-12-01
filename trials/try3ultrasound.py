import RPi.GPIO as GPIO
import time
global pulse1_start
global pulse2_start
global pulse3_start
global pulse1_end
global pulse2_end
global pulse3_end
global flag

#pulse1_start = 0;
#pulse2_start = 0;
#pulse3_start = 0;
#pulse3_end = 0;
#pulse2_end = 0;
#pulse1_end = 0;

GPIO.setmode(GPIO.BCM)


Trig  = 20 #Associate pin 23 to TRIG
Echo1 = 21 #Associate pin 24 to ECHO
Echo2 = 26 #pin 37 as led
Echo3 = 19 #pin 35 as button 

def setup():
    global flag
    flag = False;
    GPIO.setup(Trig,GPIO.OUT)
    GPIO.setup(Echo1,GPIO.IN)
    GPIO.setup(Echo2,GPIO.IN)
    GPIO.setup(Echo3,GPIO.IN)

def echo1callback(channel):
    flag = True;
    val = GPIO.input(Echo1);
    if val == 1:
        global pulse1_start
        pulse1_start= time.time()
    elif val == 0:
        global pulse1_end;
        pulse1_end=time.time()     

def echo2callback(channel):
    flag = True;
    val = GPIO.input(Echo2);
    if val == 1:
        global pulse2_start
        pulse2_start= time.time()
    elif val == 0:
        global pulse2_end;
        pulse2_end=time.time()     

def echo3callback(channel):
    flag = True;
    val = GPIO.input(Echo3);
    if val == 1:
        global pulse3_start
        pulse3_start= time.time()
    elif val == 0:
        global pulse3_end;
        pulse3_end=time.time()     

def run():
        
    global pulse1_start
    global pulse2_start
    global pulse3_start
    global pulse1_end
    global pulse2_end
    global pulse3_end
    global flag
    

    setup()

    GPIO.add_event_detect(Echo1, GPIO.BOTH, callback=echo1callback) 
    GPIO.add_event_detect(Echo2, GPIO.BOTH, callback=echo2callback)
    GPIO.add_event_detect(Echo3, GPIO.BOTH, callback=echo3callback)
    
    
    while True:
        flag = False;
        GPIO.output(Trig, True)                  #Set TRIG as HIGH
        time.sleep(0.00001)                      #Delay of 0.00001 seconds
        GPIO.output(Trig, False) 
        time.sleep(0.5); #delay between 2 triggers
          
        if(flag == True):
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
        else:
            print("flag is false");

run();
