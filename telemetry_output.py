import rospy
from clover import srv
from std_srvs.srv import Trigger
from std_msgs.msg import String

rospy.init_node('debug')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
land = rospy.ServiceProxy('land', Trigger)
telemetry_pub = rospy.Publisher("/telemetry_topic", String, queue_size=10)

r = rospy.Rate(5)
while not rospy.is_shutdown():
    telemetry = get_telemetry(frame_id='aruco_map')
    print(telemetry.cell_voltage)
    if telemetry.cell_voltage < 3.1:
        land()
        rospy.signal_shutdown("Cell voltage is too low")
    telemetry_pub.publish(f"{telemetry.x} {telemetry.y} {telemetry.z} {telemetry.roll} {telemetry.pitch} {telemetry.yaw}")
    r.sleep()