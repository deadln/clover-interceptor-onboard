#!/usr/bin/env python
import rospy
from clover import srv
from std_srvs.srv import Trigger
from std_msgs.msg import String

import numpy as np
import math
import random


node_name = "copter_node"

class CopterController():
    def __init__(self):
        rospy.init_node(node_name)
        rospy.loginfo(node_name + " started")

        self.__get_telemetry__ = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.__navigate__ = rospy.ServiceProxy('navigate', srv.Navigate)
        self.__set_position__ = rospy.ServiceProxy('set_position', srv.SetPosition)
        self.__set_velocity__ = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
        self.__land__ = rospy.ServiceProxy('land', Trigger)

        rospy.on_shutdown(self.on_shutdown_cb)

        self.FREQUENCY = 10
        self.X_VECT = np.array([1, 0, 0])
        self.SPIN_TIME = 8
        self.SPIN_RATE = math.pi / self.SPIN_TIME
        self.PATROL_SPEED = 0.3
        self.INTERCEPTION_SPEED = 0.5

        # TODO: парсить данные о полётной зоне из txt или launch файла, сделать на три координаты
        self.low_left_corner = np.array([0.0, 0.0])
        self.up_right_corner = np.array([3.0, 6.0])
        self.min_height = 0.8
        self.max_height = 4.2
        self.state = ""
        self.patrol_target = None
        self.spin_start = None
        self.pursuit_target = None
        self.pursuit_target_detections = []

    def get_telemetry(self, frame_id='aruco_map'):
        telemetry = self.__get_telemetry__(frame_id=frame_id)
        return telemetry

    def get_position(self, frame_id='aruco_map'):
        telemetry = self.__get_telemetry__(frame_id=frame_id)
        return np.array([telemetry.x, telemetry.y, telemetry.z])

    def navigate(self, target=np.array([0, 0, 2]), speed=0.5, yaw=float('nan'), yaw_rate=0.0, auto_arm=False, frame_id='aruco_map'):
        self.__navigate__(x=target[0], y=target[1], z=target[2], speed=speed, yaw=yaw, yaw_rate=yaw_rate, auto_arm=auto_arm, frame_id=frame_id)

    def set_position(self, target=np.array([0, 0, 2]), speed=0.5, yaw=float('nan'), yaw_rate=0, auto_arm=False, frame_id='aruco_map'):
        self.__set_position__(x=target[0], y=target[1], z=target[2], speed=speed, yaw=yaw, yaw_rate=yaw_rate, auto_arm=auto_arm, frame_id=frame_id)

    def set_velocity(self, target=np.array([0, 0, 0]), yaw=float('nan'), yaw_rate=0, auto_arm=False, frame_id='aruco_map'):
        self.__set_velocity__(vx=target[0], vy=target[1], vz=target[2], yaw=yaw, yaw_rate=yaw_rate, auto_arm=auto_arm, frame_id=frame_id)

    def land(self):
        return self.__land__()

    def offboard_loop(self):
        self.takeoff()

        rate = rospy.Rate(self.FREQUENCY)
        while True:  # not rospy.is_shutdown():
            if not self.is_inside_patrol_zone():
                self.return_to_patrol_zone()
            if self.state == "patrol_navigate":  # Полёт к точке патрулирования
                if self.patrol_target is None:
                    self.set_patrol_target()
                    # self.navigate(self.patrol_target[0], self.patrol_target[1], self.patrol_target[2], self.get_yaw_angle(self.X_VECT, self.patrol_target))
                    self.navigate(self.patrol_target, yaw=float('nan'), yaw_rate=self.SPIN_RATE)
                elif self.is_navigate_target_reached():
                    self.patrol_target = None
                    # self.state = "patrol_spin"

            # if self.state == "patrol_spin":  # Вращение в точке патрулирования
            #     if self.spin_start is None:
            #         self.spin_start = rospy.get_time()
            #         self.navigate(yaw=float('nan'), yaw_rate=self.SPIN_RATE)
            #     elif rospy.get_time() - self.spin_start >= self.SPIN_TIME:
            #         self.spin_start = None
            #         self.state = "patrol_navigate"

            if self.state == "pursuit":  # Состояние преследования цели, которая однозначно обнаружена
                position = self.get_position(frame_id='aruco_map')
                error = self.pursuit_target - position
                velocity = error / np.linalg.norm(error) * self.INTERCEPTION_SPEED
                self.set_velocity(velocity, yaw=self.get_yaw_angle(self.X_VECT, self.pursuit_target))

            if self.state == "suspicion":  # Проверка места, в котором с т.з. нейросети "мелькнул дрон"
                pass
            if self.state == "search":  # Поиск утерянной цели
                pass

            if self.state == "rtb":  # Возвращение на базу
                pass

            # rate.sleep()

    def takeoff(self):
        self.navigate(frame_id="", auto_arm = True)
        rospy.sleep(5)
        self.state = "patrol_navigate"

    def navigate_wait(self, x=0, y=0, z=2, yaw=float('nan'), speed=0.2, frame_id='aruco_map', auto_arm=False, tolerance=0.3):
        self.navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

        while not rospy.is_shutdown():
            if self.is_navigate_target_reached(tolerance):
                break
            rospy.sleep(0.2)

    def set_patrol_target(self):
        self.patrol_target = np.array([random.uniform(self.low_left_corner[0], self.up_right_corner[0]),
                         random.uniform(self.low_left_corner[1], self.up_right_corner[1]),
                         random.uniform(self.min_height, self.max_height)])

    def get_yaw_angle(self, vector_1, vector_2):
        unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
        unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
        dot_product = np.dot(unit_vector_1, unit_vector_2)
        angle = np.arccos(dot_product)
        if vector_2[1] < 0:
            angle *= -1
        return angle

    def return_to_patrol_zone(self):  # TODO: сделать на три координаты
        position = self.get_position()
        velocity = np.zeros(2)
        velocity += list(map(int, position < self.low_left_corner))
        velocity += list(map(int, position > self.up_right_corner))
        velocity *= self.INTERCEPTION_SPEED
        self.set_velocity(velocity)

    def is_navigate_target_reached(self, tolerance):
        position = self.get_position(frame_id='navigate_target')
        return np.linalg.norm(position) < tolerance
        # return math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance

    def is_inside_patrol_zone(self):
        position = self.get_position()
        return all(position >= self.low_left_corner) and all(position <= self.up_right_corner)

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