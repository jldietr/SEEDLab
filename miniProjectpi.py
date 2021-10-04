from picamera import PiCamera
from time import sleep
from statistics import mean
import numpy as np
import cv2 as cv
import RPi.GPIO as GPIO

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
    
    if (cX > width)and(cY > height):
        outputTwo = 0
        outputOne = 1
    elif (cX > width)and(cY < height):
        outputTwo = 0
        outputOne = 0
    elif (cX < width)and(cY < height):
        outputTwo = 1
        outputOne = 1
    elif (cX < width)and(cY > height):
        outputTwo = 1
        outputOne = 0
    else:
        None
    
    #simple code to make calling the center easier
    center = (cX,cY)
    
    #Print x and y location of yellow object center if on screen
    #Show images in output window with center marker, wait for key press and close windows
    if center != None:
        
        GPIO.output(23, outputOne)
        GPIO.output(24, outputTwo)       
        
        print ("Output: ",outputTwo,outputOne)
    else:
        print("No Markers Found")
