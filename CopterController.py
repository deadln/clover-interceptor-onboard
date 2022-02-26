#!/usr/bin/env python
import math
import rospy
from clover import srv
from std_srvs.srv import Trigger
import numpy as np
from std_msgs.msg import String

FREQ = 10

node_name = "copter_node"

class CopterController():
    def __init__(self):
        rospy.init_node(node_name)
        rospy.loginfo(node_name + " started")

        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        self.navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
        self.set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
        self.set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
        self.set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
        self.set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
        self.land = rospy.ServiceProxy('land', Trigger)

        rospy.on_shutdown(self.on_shutdown_cb)

        self.state = ""
        self.patrol_target = None

    def offboard_loop(self):
        self.takeoff()

        rate = rospy.Rate(FREQ)
        while not rospy.is_shutdown():
            if self.state == "patrol_navigate":
                

            rate.sleep()

    def takeoff(self):
        self.navigate(frame_id="", auto_arm = True)
        rospy.sleep(5)
        self.state = "patrol_navigate"

    def navigate_wait(self, x=0, y=0, z=2, yaw=float('nan'), speed=0.2, frame_id='aruco_map', auto_arm=False, tolerance=0.3):
        self.navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

        while not rospy.is_shutdown():
            telem = self.get_telemetry(frame_id='navigate_target')
            if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
                break
            rospy.sleep(0.2)

    def on_shutdown_cb(self):
        rospy.logwarn("shutdown")
        self.land()
        rospy.loginfo("landing complete")


if __name__ == '__main__':
    controller = CopterController()
    try:
        CopterController.offboard_loop()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()