#computer vision imports
from picamera import PiCamera
from time import sleep
from statistics import mean
import numpy as np
import cv2 as cv
import RPi.GPIO as GPIO
#system integration imports
import smbus
from smbus2 import SMBus
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

#LCD initialization
lcd_columns = 16
lcd_rows = 2
i2c = board.I2C()
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.clear()
lcd.text_direction = lcd.LEFT_TO_RIGHT
#I2C setup
bus = smbus.SMBus(1)
address = 0x04

        
def writeNumber(value):
    bus.write_byte_data(address,0,value)
    return -1

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)


# this is from the cameras perspective 
# Upper right: ouput will be 00 for Pi/2
# Bottom right: output will be 01 for pi
# Bottom left: ouput will be 10 for 3pi/2
# Top left: ouput will be 11 for 0pi
# outputOne --> LSB --> GPIO 23

    
#load camera start preveiw and capture pic as Ex1.jpg
#Set lower and uper limits for range functionand create kernal
low = (96,150,100)
high = (108,255,255)
kernal = np.ones((5,5),np.uint8)
camera = PiCamera()
camera.resolution = (320,240)
camera.framerate = 24
sleep(2)
img = np.empty((240,320,3), dtype=np.uint8)

while True:
   
    
    camera.capture(img,'bgr')
    Filter = cv.GaussianBlur(img,(5,5),0)
    width = (img.shape[1])/2
    height = (img.shape[0])/2
    
    outputOne = 0;
    outputTwo = 0;
    
    #convert res image to HSV, create mask and put mask over original res image
    imgHSV = cv.cvtColor(Filter, cv.COLOR_BGR2HSV)
    mask = cv.inRange(imgHSV, low, high)
    mask = cv.morphologyEx(mask,cv.MORPH_OPEN,kernal) #using open morphology to get rid of dots
    yellow = cv.bitwise_and(img, img, mask = mask)
    
    #Use threshold on mask to only get yellow object
    ret,thresh = cv.threshold(mask,127,255,0)
    
    #Set x and y arrays to non zero values from threshold tuple
    Y,X= np.nonzero(thresh)
    cX = X.mean()
    cY = Y.mean()
    
    # if there is blue tape find the angle with respect to the camera
    if cX != 0:
            
        phi = (27 * (cX-width))/width #calculating phi with picture width
        phi = round(phi,2)
        phi = str(phi)
        lcd.message = "Angle: " + phi + "  "
        if len(phi) < 6:
            digits = 6 - len(phi)
            while digits != 0:
                phi = phi + "0"
                digits = 6 - len(phi)
        for i in phi:
            print(ord(i))
            writeNumber(ord(i))
            
    else:
            
        print("No Markers Found")
        lcd.message = "No Angle Found   "
        phi = "180.00"
        for i in phi:
                print(ord(i))
                writeNumber(ord(i))
