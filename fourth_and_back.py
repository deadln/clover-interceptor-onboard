import math
import rospy
from clover import srv
from std_srvs.srv import Trigger
import numpy as np

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

p2 = np.array([1.0, 1.0, 2.0])
p1 = np.array([3.0, 3.0, 2.0])

def navigate_wait(x=0, y=0, z=2, yaw=float('nan'), speed=0.2, frame_id='aruco_map', auto_arm=False, tolerance=0.3):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

# print(navigate(x=0, y=0, z=2, frame_id="", auto_arm = True))
set_velocity(vx=0, vy=0, vz=0.3, yaw=float('nan'), frame_id="body", auto_arm=True)
rospy.sleep(4)
print("took off")

land_zone = get_telemetry(frame_id='aruco_map')

# navigate_wait(p1[0], p1[1], p1[2])
navigate_wait(p1[0], p1[1], p1[2])
print("point reached")
navigate_wait(land_zone.x, land_zone.y, land_zone.z)
print("land zone reached")
land()