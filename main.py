#!/usr/bin/env python
import math
import rospy
from clover import srv
from std_srvs.srv import Trigger
import numpy as np
from std_msgs.msg import String

node_name = "main_node"


def offboard_loop():
    pass


def on_shutdown_cb():
    rospy.logwarn("shutdown")
    land()
    rospy.loginfo("landing complete")


if __name__ == '__main__':
    rospy.init_node(node_name)
    rospy.loginfo(node_name + " started")

    get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
    navigate = rospy.ServiceProxy('navigate', srv.Navigate)
    navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
    set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
    set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
    set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
    set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
    land = rospy.ServiceProxy('land', Trigger)


    rospy.on_shutdown(on_shutdown_cb)

    try:
        offboard_loop()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()