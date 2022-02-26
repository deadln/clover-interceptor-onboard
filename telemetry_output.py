import rospy
from clover import srv
from std_srvs.srv import Trigger

rospy.init_node('debug')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)

r = rospy.Rate(5)
while not rospy.is_shutdown():
    print(get_telemetry(frame_id='aruco_map'))
    r.sleep()