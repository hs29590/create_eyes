import RPi.GPIO as GPIO                    #Import GPIO library

import time                                #Import time library

global Ultrasoundswitch=0			   #global declaration

GPIO.setmode(GPIO.BCM)                     #Set GPIO pin numbering 



TRIG = 20 #Associate pin 23 to TRIG

ECHO = 21                                  #Associate pin 24 to ECHO

LED= 26                                    #pin 37 as led

BUTTON= 19    #pin 35 as button 



def Setup():

    GPIO.setup(TRIG,GPIO.OUT)                  #Set pin as GPIO out

    GPIO.setup(ECHO,GPIO.IN)                   #Set pin as GPIO in

    GPIO.setup(LED,GPIO.OUT)                     #Set pin as GPIO out

    GPIO.setup(BUTTON,GPIO.IN,pull_up_down=GPIO.PUD_UP)                     #Set pin as GPIO in and pull up



def loop():

    BUTTON_State=GPIO.input(BUTTON)

    if BUTTON_State==False: #when button pressed then 

       Ultrasoundswitch+=1 #this code is used for 

       Ultrasoundswitch%=2 #toggling flag(ultrasoundswitch between 0 and 1		    







    if Ultrasoundswitch==1:   #execcute only if button is pressed

       GPIO.output(TRIG, False)                 #Set TRIG as LOW

       time.sleep(2)                            #Delay of 2 seconds(just to be safe)



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



       if distance > 10 and distance < 25:      #Check whether the distance is within range

          GPIO.output(LED, True)  #LED HIGH 

       else:

          GPIO.output(LED, False)                   #LED LOW



    if Ultrasoundswitch==0 #if toggled to off then 

       pulse_duration=0   #reset pulse duration  

       distance=0         #reset distnace

       GPIO.output(LED, False)



setup()

while True: 

      loop()


