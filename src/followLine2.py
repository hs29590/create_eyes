#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('create_eyes')
import timeit
import sys
import rospy
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
import numpy
import math


class line_extractor:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/raspicam_node/image_raw",Image,self.callback)

    #print("Param is", rospy.get_param('default_param', 'default_value'));
    show_images_from_param = rospy.get_param('~show_images', 'False')
    rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~show_images'), show_images_from_param)
    self.showImages = bool(show_images_from_param);

    use_pid_from_param = rospy.get_param('~use_pid', 'True');
    self.use_pid = bool(use_pid_from_param);

    self.image = None;
    self.SCALE_FACTOR = 4;
    self.gray_image = None;
    self.prevCx = None;
    self.direction = 0
    self.lineFound = False;
    self.err_pub = rospy.Publisher('line_error', Float32, queue_size=1)
    self.pid_state_pub = rospy.Publisher('state', Float64, queue_size=1)
    self.pid_setpoint_pub = rospy.Publisher('setpoint', Float64, queue_size=1)
    self.line_state_pub = rospy.Publisher('line_visible', Bool, queue_size = 1);

    self.kernel = numpy.ones((3,3),numpy.uint8)
    self.lower_sat = numpy.array([200])
    self.upper_sat = numpy.array([255])
    
    self.lower_yellow = numpy.uint8([0, 120, 120])
    self.upper_yellow = numpy.uint8([255, 255, 255])

    self.twist = Twist()

  def callback(self,msg):

    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    image = cv2.resize(image,(int(320/self.SCALE_FACTOR),int(240/self.SCALE_FACTOR)));
    
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hue_img, sat_img, v_img = cv2.split(hsv)  # extracting red channel
    
    sat_mask = cv2.inRange(sat_img, self.lower_sat, self.upper_sat);

    yellow_mask = cv2.inRange(image, self.lower_yellow, self.upper_yellow);
    
    if(self.showImages):
        cv2.imshow('sat',sat_mask);
        cv2.waitKey(3);

    masked = cv2.bitwise_and(sat_mask, sat_mask, mask = yellow_mask)

    masked= cv2.GaussianBlur(masked,(3,3),2)#blur the gray scale image
    masked = cv2.morphologyEx(masked,cv2.MORPH_ERODE, self.kernel)
    if(self.showImages):
        cv2.imshow('y',masked);
        cv2.waitKey(3)
    mask = masked;

    h, w, d = image.shape

    if(self.use_pid):
        self.pid_setpoint_pub.publish(float(w)/2.0);

    search_top = 0;
    search_bot = 0.6*h;
    search_left = w/4;
    search_right = 3*w/4;
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    mask[0:h, 0:search_left] = 0
    mask[0:h, search_right:w] = 0

    if(self.showImages):
        cv2.imshow("a",mask);
        cv2.waitKey(3);

    M = cv2.moments(mask)
    if M['m00'] > 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        
        if(self.use_pid):
            self.pid_state_pub.publish(float(cx));
        
        cv2.circle(image, (int(cx), int(cy)), 5, (0,0,255), -1)  

        if self.prevCx is None:
            self.prevCx = cx;

        #smoothing cx
        cx = self.prevCx*0.5 + cx*0.5;
        self.prevCx = cx;
        err = cx - w/2
        err = self.SCALE_FACTOR * err;
        self.line_state_pub.publish(True);
        if(not self.use_pid):
            self.err_pub.publish(err);

    else: #Moment not available. Probably, line isn't there
        err = -1000.0;
        self.line_state_pub.publish(False);
        self.err_pub.publish(err);
    
    if(self.showImages):
        cv2.imshow("window", image)
        cv2.waitKey(3)

def main(args):
  rospy.init_node('line_extractor', anonymous=True)
  ic = line_extractor()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
