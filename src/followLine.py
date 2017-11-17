#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('create_eyes')
import sys
import rospy
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy
import math


class line_extractor:


  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
  
    self.image = None;
    self.gray_image = None;
    self.prevCx = None;
    self.direction = 0
    self.lineFound = False;
    #self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)


    self.noLineCount = 0;
    self.twist = Twist()

  def preprocessImage(self,img):
    kernel = numpy.ones((7,7),np.uint8)

    #self.gray_image = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)#convert each frame to grayscale.
    #clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    #gray = clahe.apply(self.gray_image)
    #self.gray_image = img[:,:,2];
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV);
    hue_img, sat_img, v_img = cv2.split(hsv)  # extracting red channel
    gray = sat_img;
    self.gray_image = gray;
    #gray = self.gray_image;
    blur=cv2.GaussianBlur(gray,(9,9),2)#blur the grayscale image
    #blur=cv2.GaussianBlur(blur,(9,9),2)#blur the grayscale image
    cv2.imshow("aa", blur);
    cv2.waitKey(3);
    #ret,th1 = cv2.threshold(blur,70,255,cv2.THRESH_BINARY)#using threshold remove noise
    ret,th1 = cv2.threshold(blur,0,255,cv2.THRESH_OTSU)#using threshold remove noise
    #ret,th1 = cv2.threshold(blur,60,255,cv2.THRESH_BINARY_INV)#using threshold remove noise
    closing = cv2.morphologyEx(th1, cv2.MORPH_CLOSE, kernel)
    closing = cv2.morphologyEx(closing, cv2.MORPH_CLOSE, kernel)
    return closing;


  def selectLineContour(self,contours):
       
    if(len(contours) == 0):
      return None;

    cntcolors = [ (0,255,0), (255, 0, 0), (0, 0, 255), (255,255,0), (0,255,255), (255,0,255), (128,128,0), (128,0,128), (0, 128, 128), (128, 0, 0), (0, 128, 0),  (0, 0, 128) ];
    cntcolornames = [ 'green', 'red', 'blue', 'yellow', 'cyan', 'magenta', 'light yellow', 'light magenta', 'light cyan', 'light red', 'light green', 'light blue'];

    lineContour = None;
    i = 0;

    lowest_mean_value = 10000;

    for cnt in contours:
       if cnt is not None:
           area = cv2.contourArea(cnt)# find the area of contour
           mask = numpy.zeros(self.gray_image.shape,numpy.uint8)
           cv2.drawContours(mask,cnt,0,255,-1)
           mean_val = cv2.mean(self.gray_image,mask = mask)
           x,y,w,h = cv2.boundingRect(cnt)
           #print ("Width: " + str(w))
           #cv2.drawContours(self.image,cnt,-1,cntcolors[0],3)
           #cv2.waitKey(3);
           dist_from_center = math.sqrt( math.pow((x+w/2) - (self.image.shape[1]/2), 2) + math.pow((y+h/2)- (self.image.shape[0]/2),2));
           #print("Area" + str(area) + " w " + str(w) + "Dist from center: " + str(dist_from_center));
           if(area > 500 and w > 20 and dist_from_center < 100):
               if(lowest_mean_value > mean_val[0]):
                   #print("HERE");
                   lowest_mean_value = mean_val[0];
                   lineContour = cnt;
          
    x,y,w,h = cv2.boundingRect(lineContour)
    cv2.drawContours(self.image,lineContour,-1,cntcolors[0],3)
    self.image = cv2.rectangle(self.image,(x,y),(x+w,y+h),(255,0,0),3)
     #printstr = "Color: " + str(cntcolornames[i%11]) + "\nArea: " + str(area) + "\nMean: " + str(mean_val) + "\nCx: " + str(x + w/2) + " Cy: " + str(y + h/2);
     #printstr = printstr + "\nARatio: " + str(float(w)/h) + "\nAngle: " + str(angle);
     #      print (  printstr),
     #      print ("------------------");
     #  i = i + 1;    
    cv2.imshow("cropped", self.image)
    cv2.waitKey(3)

    return lineContour;
            
  def getDirection(self,lineContour):

    direction = None;  

    #calculate direction from the contour we think is the line
    if(lineContour is None):
        return direction;
    
    #try:
    (x,y),(MA,ma),angle = cv2.fitEllipse(lineContour);
    direction = angle;
    
#    print("Area: " + str(cv2.contourArea(lineContour)));
    
    #except:
    #    x,y,w,h = cv2.boundingRect(lineContour)
    #    myradians = math.atan2((y+h/2) - self.image.shape[0], (x+w/2) - self.image.shape[1]/2)
    #    direction = math.degrees(myradians)
    #    print("USING CENTER ANGLE\n\n\n");

    return direction;

  def callback(self,msg):
    kernel = numpy.ones((7,7),numpy.uint8)
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    
    scaled_image = cv2.resize(image,(320,240))
    crop_img = scaled_image[180:240, 0:320] # Crop from x, y, w, h -> 100, 200, 300, 400

    #self.image = crop_img;

    
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hue_img, sat_img, v_img = cv2.split(hsv)  # extracting red channel
    
    gray = sat_img;
    self.gray_image = gray;
  
    blur=cv2.GaussianBlur(gray,(9,9),2)#blur the grayscale image
  
    cv2.imshow("aa", blur);
    cv2.waitKey(3);
  
    ret,th1 = cv2.threshold(blur,0,255,cv2.THRESH_OTSU)#using threshold remove noise
  
    closing = cv2.morphologyEx(th1, cv2.MORPH_CLOSE, kernel)
    closing = cv2.morphologyEx(closing, cv2.MORPH_CLOSE, kernel)
    
    cv2.imshow("hs",closing);
    cv2.waitKey(3)
    lower_yellow = numpy.array([ 25, 106, 120])
    upper_yellow = numpy.array([62, 174, 250])  
    #mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask = closing;
    

    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + h/4;
    search_left = w/3;
    search_right = 2*w/3;
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    mask[0:h, 0:search_left] = 0
    mask[0:h, search_right:w] = 0
    cv2.imshow("ae",mask);
    cv2.waitKey(3)
    M = cv2.moments(mask)
    if M['m00'] > 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        
        #The proportional controller is implemented in the following four lines which
        #is reposible of linear scaling of an error to drive the control output.
        
        if self.prevCx is None:
            self.prevCx = cx;
            
        #smoothing cx
        cx = self.prevCx*0.5 + cx*0.5;
        
        self.prevCx = cx;
        cv2.circle(image, (int(cx), int(cy)), 20, (0,0,255), -1)  
        err = cx - w/2
        self.twist.linear.x = 0.2
        self.twist.angular.z = -float(err) / 100
        self.cmd_vel_pub.publish(self.twist)
        self.noLineCount = 0;
    else: #Moment not available. Probably, line isn't there
        self.noLineCount = self.noLineCount + 1;
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        if(self.noLineCount >= 10):
            self.cmd_vel_pub.publish(self.twist)
    cv2.imshow("window", image)
    cv2.waitKey(3)

def main(args):
  ic = line_extractor()
  rospy.init_node('line_extractor', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

