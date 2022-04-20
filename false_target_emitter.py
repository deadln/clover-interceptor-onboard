import rospy
from std_msgs.msg import String
from tkinter import *
from tkinter import ttk
import threading

def array_to_string(arr):
    s = ''
    for a in arr:
        s += str(a) + ' '
    s = s.strip()
    return s

def engage_target():
    global emit
    emit = True
    pub.publish(target)

def disengage_target():
    global emit
    emit = False
    pub.publish('')

# def ros_loop():
#     r = rospy.Rate(5)
#     while not rospy.is_shutdown():
#
#         if emit:
#         else:
#         r.sleep()

target = '2.0 2.0 1.3'
emit = False
rospy.init_node('false_target')

pub = rospy.Publisher("drone_detection/false_target", String, queue_size=10)

win = Tk()
win.geometry("400x400")
win.title("Ложная цель")
engage_button = ttk.Button(win, text="Отобразить ложную цель", command=engage_target)
disengage_button = ttk.Button(win, text="Убрать ложную цель", command=disengage_target)
engage_button.pack()
disengage_button.pack()

# thread = threading.Thread(target=ros_loop)
# thread.start()

win.mainloop()