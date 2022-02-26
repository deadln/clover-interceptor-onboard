#!/usr/bin/env python
import math
import rospy
from clover import srv
from std_srvs.srv import Trigger
import numpy as np
from std_msgs.msg import String
from CopterController import *

if __name__ == '__main__':
    controller = CopterController()
    try:
        CopterController.offboard_loop()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()