'''
* Author: Thomas W. Talbot
* Date Modified: 09/09/21
* The purpose of this program is to read in a PIR sensor from the Beagle Bone
* Black continuously.  
          
'''      
          
import Adafruit_BBIO.GPIO as GPIO
import time

PIR_PIN = "P8_8"
GPIO.setup(PIR_PIN, GPIO.IN)
GPIO.add_event_detect(PIR_PIN, GPIO.RISING) 
    
while True:
    if GPIO.event_detected("P8_8"):
        print("Motion Detected!")
        time.sleep(0.5)
    else:
        print("You are not moving!")
        time.sleep(0.5)