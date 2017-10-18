#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('create_eyes')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import numpy as np
import pdb
import time

from ImageProcessor import *
from Utils import *


class line_extractor:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
    
    self.font = cv2.FONT_HERSHEY_SIMPLEX
    self.direction = 0
    self.Images=[]
    self.N_SLICES = 4


  def process1(self,in_image):
    out_image = in_image;  
    return out_image;

  def process2(self,in_image):
    imgray = cv2.cvtColor(in_image,cv2.COLOR_BGR2GRAY) #Convert to Gray Scale
    ret, thresh = cv2.threshold(imgray,100,255, cv2.THRESH_BINARY_INV) #Get Threshold

    kernel = np.ones((5,5),np.uint8)
    dilation = cv2.dilate(thresh,kernel,iterations = 1)
    out_image = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    return out_image;

  def process3(self,in_image):
      # Convert BGR to HSV
    hsv = cv2.cvtColor(in_image, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    lower_blue = np.array([110,50,50])
    upper_blue = np.array([130,255,255])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    
    out_image = cv2.bitwise_and(in_image,in_image, mask= mask)
    
    return out_image;

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    self.direction = 0
    self.Images=[]
    for q in range(self.N_SLICES):
        self.Images.append(ImageProcessor())

    (rows,cols,channels) = cv_image.shape
   
    #scaled_image = cv2.resize(cv_image,(320,240))

    cv2.imshow("Process3", self.process3(cv_image))
    cv2.imshow("Proces2",self.process2(cv_image))
    cv2.waitKey(3)


    #crop_img = scaled_image[120:240, 0:320] # Crop from x, y, w, h -> 100, 200, 300, 400
    # NOTE: its img[y: y + h, x: x + w] and *not* img[x: x + w, y: y + h]
    #cv2.imshow("cropped", crop_img)
    #cv2.waitKey(3)

    #gray=cv2.cvtColor(crop_img,cv2.COLOR_BGR2GRAY)#convert each frame to grayscale.
    #blur=cv2.GaussianBlur(gray,(9,9),2)#blur the grayscale image
    #ret,th1 = cv2.threshold(blur,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)#using threshold remove noise
#    ret1,th2 = cv2.thresho  ild(th1,127,255,cv2.THRESH_BINARY_INV)# invert the pixels of the image frame
    #_, contours, _ = cv2.findContours(th1,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE) #find the contours
    #cv2.drawContours(crop_img,contours,-1,(0,255,0),3)
   #     cv2.imshow('frame',frame) #show video 
    #for cnt in contours:
    #   if cnt is not None:
    #       area = cv2.contourArea(cnt)# find the area of contour
    #   if area>=500 :
    #        # find moment and centroid
    #        M = cv2.moments(cnt)
    #        cx = int(M['m10']/M['m00'])
    #        cy = int(M['m01']/M['m00'])
#    self.direction = 0
#    if crop_img is not None:
        
        
#        cv2.imshow("Image Window", crop_img)
#        cv2.waitKey(3);

#    print("Direction - ");
#    print(cx,cy);
    
#    cv2.imshow("Image window", crop_img)
#    cv2.waitKey(3)

#    try:
#      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
#    except CvBridgeError as e:
#      print(e)

def main(args):
  ic = line_extractor()
  rospy.init_node('try2', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

