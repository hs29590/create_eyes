#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('create_eyes')
import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

import numpy
import math


class Create2StatePublisher:

  def __init__(self):

    self.sonar_sub = rospy.Subscriber('line_visible', Bool, self.sonarCallback)
    self.line_sub = rospy.Subscriber('sonar_state', Bool, self.lineCallback)
    self.gui_sub = rospy.Subscriber('gui_state', String, self.guiCallback)

    self.master_state_pub = rospy.Publisher('master_state', String, queue_size=1)

    self.last_gui_state = "Stop";
    

def main(args):
  rospy.init_node('create_eyes_state_publisher', anonymous=True)
  ic = DriveCreate2()
  rospy.spin();
        
if __name__ == '__main__':
    main(sys.argv)

