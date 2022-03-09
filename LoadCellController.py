import RPi.GPIO as gpio
import time
import rospy
from std_msgs.msg import Bool

DT = 27
SCK = 17
DRONE_WEIGHT = 100
WEIGHT_DELIMETER = 283

node_name = "load_cell_node"

def readCount():
    i = 0
    Count = 0
    gpio.setup(DT, gpio.OUT)
    gpio.setup(SCK, gpio.OUT)
    gpio.output(DT, 1)
    gpio.output(SCK, 0)
    gpio.setup(DT, gpio.IN)

    while gpio.input(DT) == 1:
        i = 0
    for i in range(24):
        gpio.output(SCK, 1)
        Count = Count << 1

        gpio.output(SCK, 0)
        # time.sleep(0.001)
        if gpio.input(DT) == 0:
            Count = Count + 1

    gpio.output(SCK, 1)
    Count = Count ^ 0x800000
    gpio.output(SCK, 0)
    return Count


if __name__ == '__main__':
    rospy.init_node(node_name)
    rospy.loginfo(node_name + " started")
    pub = rospy.Publisher("load_cell", Bool, queue_size=10)

    gpio.setmode(gpio.BCM)
    sample = readCount()
    try:
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            count = readCount()
            weight = (sample - count) / WEIGHT_DELIMETER
            print(weight)
            if weight >= DRONE_WEIGHT:
                pub.publish(True)
            else:
                pub.publish(False)
            # print(1)
            r.sleep()
    except KeyboardInterrupt:
        print("Stop")



