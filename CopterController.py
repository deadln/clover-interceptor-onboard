#!/usr/bin/env python
import rospy
from clover import srv
from std_srvs.srv import Trigger
from std_msgs.msg import String
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge

import numpy as np
import math
import random
from enum import Enum


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

        self.bridge = CvBridge()

        self.FREQUENCY = 5
        self.DEPTH_QUEUE_SIZE = 500
        self.CAMERA_ANGLE_H = 1.5009831567151235
        self.CAMERA_ANGLE_V = 0.9948376736367679
        self.CRITICAL_CELL_VOLTAGE = 3.07
        self.SUSPICION_DURATION = 5
        self.PURSUIT_DURATION = 2
        self.SEARCH_DURATION = 5
        self.SUSPICION_TRIGGER_COUNT = 5
        self.PURSUIT_TRIGGER_COUNT = 10


        self.X_NORM = np.array([1, 0, 0])
        self.SPIN_TIME = 8
        self.SPIN_RATE = 2 * math.pi / self.SPIN_TIME
        self.PATROL_SPEED = 0.3
        self.INTERCEPTION_SPEED = 0.5
        self.DETECTION_DIAPASON_SEC = 1.0

        # TODO: парсить данные о полётной зоне из txt или launch файла
        self.low_left_corner = np.array([2.5, 0.5, 0.5])
        self.up_right_corner = np.array([7.0, 4.5, 3.2])
        self.telemetry = None
        self.state = ""
        self.state_timestamp = rospy.get_time()
        self.patrol_target = None
        self.spin_start = None
        self.consecutive_detections = 0
        self.suspicion_target = None
        self.pursuit_target = None
        self.pursuit_target_detections = []
        self.depth_images = []

        # self.depth_debug = rospy.Publisher("debug/depth", Image, queue_size=10)
        # self.target_local_debug = rospy.Publisher("debug/target_position_local", PointCloud, queue_size=10)
        # self.target_global_debug = rospy.Publisher("debug/target_position_global", PointCloud, queue_size=10)
        # rospy.Subscriber('drone_detection/target', String, self.target_callback)
        self.telemetry_pub = rospy.Publisher("/telemetry_topic", String, queue_size=10)
        rospy.Subscriber("drone_detection/target_position", Point32, self.target_callback)
        # rospy.Subscriber('drone_detection/false_target', String, self.target_callback_test)  # TODO: протестировать реакцию на ложную цель
        # rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_image_callback)

        rospy.on_shutdown(self.on_shutdown_cb)

    def get_telemetry(self, frame_id='aruco_map'):
        if frame_id == 'aruco_map':
            return self.telemetry
        telemetry = self.__get_telemetry__(frame_id=frame_id)
        return telemetry

    def get_position(self, frame_id='aruco_map'):
        if frame_id == 'aruco_map':
            return np.array([self.telemetry.x, self.telemetry.y, self.telemetry.z])
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
        while not rospy.is_shutdown():
            self.telemetry = self.__get_telemetry__(frame_id='aruco_map')
            self.telemetry_pub.publish(
                f"{self.telemetry.x} {self.telemetry.y} {self.telemetry.z} {self.telemetry.roll} {self.telemetry.pitch} {self.telemetry.yaw}")
            if self.telemetry.cell_voltage < self.CRITICAL_CELL_VOLTAGE:
                rospy.logfatal("CRITICAL CELL VOLTAGE: {}".format(self.telemetry.cell_voltage))
                rospy.signal_shutdown("Cell voltage is too low")
            self.check_state_duration()
            if not self.is_inside_patrol_zone():
                self.return_to_patrol_zone()
                continue
            if self.state == State.PATROL_NAVIGATE:  # Полёт к точке патрулирования
                if self.patrol_target is None:
                    self.set_patrol_target()
                    rospy.loginfo(f"New patrol target {self.patrol_target}")
                else:
                    # Полёт напрямую
                    # print("YAW", self.get_yaw_angle(self.X_NORM, self.patrol_target - self.get_position()) / (math.pi / 180))
                    self.navigate(self.patrol_target, speed=self.PATROL_SPEED, yaw=self.get_yaw_angle(self.X_NORM, self.patrol_target - self.get_position()))
                    # Полёт с вращением
                    # self.navigate(self.patrol_target, speed=self.PATROL_SPEED, yaw=float('nan'), yaw_rate=self.SPIN_RATE)
                if self.is_navigate_target_reached():  # Argument: target=self.patrol_target
                    rospy.loginfo("Patrol target reached")
                    self.patrol_target = None
                    # self.state = "patrol_spin"

            # if self.state == "patrol_spin":  # Вращение в точке патрулирования
            #     if self.spin_start is None:
            #         self.spin_start = rospy.get_time()
            #         self.navigate(self.get_position(), yaw=float('nan'), yaw_rate=self.SPIN_RATE)
            #     elif rospy.get_time() - self.spin_start >= self.SPIN_TIME:
            #         self.spin_start = None
            #         self.state = "patrol_navigate"

            if self.state == State.PURSUIT:  # Состояние преследования цели, которая однозначно обнаружена
                if self.pursuit_target is None:
                    self.set_state(State.PATROL_NAVIGATE)
                else:
                    position = self.get_position(frame_id='aruco_map')
                    error = self.pursuit_target + np.array([0, 0, 0.7]) - position
                    velocity = error / np.linalg.norm(error) * self.INTERCEPTION_SPEED
                    rospy.loginfo(f"In pursuit. Interception velocity {velocity}")
                    self.set_velocity(velocity, yaw=self.get_yaw_angle(self.X_NORM, self.pursuit_target - self.get_position()))

            if self.state == State.SUSPICION:  # Проверка места, в котором с т.з. нейросети "мелькнул дрон"
                pass
            if self.state == State.SEARCH:  # Поиск утерянной цели
                pass

            if self.state == State.RTB:  # Возвращение на базу
                pass

            rate.sleep()

    def takeoff(self):
        self.set_velocity(np.array([0, 0, 0.2]), yaw=float('nan'), frame_id="body", auto_arm=True)
        # self.navigate(frame_id="", auto_arm = True)
        rospy.sleep(0.5)
        self.set_state(State.PATROL_NAVIGATE)
        rospy.loginfo("Takeoff complete")

    def navigate_wait(self, x=0, y=0, z=2, yaw=float('nan'), speed=0.2, frame_id='aruco_map', auto_arm=False, tolerance=0.3):
        self.navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

        while not rospy.is_shutdown():
            if self.is_navigate_target_reached(tolerance):
                break
            rospy.sleep(0.2)

    def set_patrol_target(self):
        self.patrol_target = self.get_position()
        while np.linalg.norm(self.patrol_target - self.get_position()) < np.linalg.norm(
                self.low_left_corner - self.up_right_corner) / 3:
            self.patrol_target = np.array([random.uniform(self.low_left_corner[0], self.up_right_corner[0]),
                                           random.uniform(self.low_left_corner[1], self.up_right_corner[1]),
                                           random.uniform(self.low_left_corner[2], self.up_right_corner[2])])

    def get_yaw_angle(self, vector_1, vector_2):
        unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
        unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
        dot_product = np.dot(unit_vector_1, unit_vector_2)
        angle = np.arccos(dot_product)
        if vector_2[1] < 0:
            angle *= -1
        return angle

    def return_to_patrol_zone(self):
        position = self.get_position()
        velocity = np.zeros(3)
        velocity += np.array(list(map(int, position < self.low_left_corner)))
        velocity += np.array(list(map(int, position > self.up_right_corner))) * -1
        velocity *= self.INTERCEPTION_SPEED
        self.set_velocity(velocity)
        rospy.logwarn(f"OUT OF PATROL ZONE. RETURN VECTOR {velocity}")

    def set_state(self, state):
        self.state = state
        rospy.loginfo("Changed state to " + state.value)
        if state == State.SUSPICION or state == State.PURSUIT or state == State.SEARCH:
            self.state_timestamp = rospy.get_time()

    def is_navigate_target_reached(self,  tolerance=0.3, target=None):
        if target is None:
            position = self.get_position(frame_id='navigate_target')
        else:
            position = target - self.get_position()
        return np.linalg.norm(position) < tolerance
        # return math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance

    def is_inside_patrol_zone(self):
        position = self.get_position()
        return all(position >= self.low_left_corner) and all(position <= self.up_right_corner)

    def check_state_duration(self):
        if self.state == State.SUSPICION and rospy.get_time() - self.state_timestamp > self.SUSPICION_DURATION:
            # self.state = State.PATROL_NAVIGATE
            self.set_state(State.PATROL_NAVIGATE)
        elif self.state == State.PURSUIT and rospy.get_time() - self.state_timestamp > self.PURSUIT_DURATION:
            # self.state = State.SEARCH
            self.set_state(State.SEARCH)
        elif self.state == State.SEARCH and rospy.get_time() - self.state_timestamp > self.SEARCH_DURATION:
            # self.state = State.PATROL_NAVIGATE
            self.set_state(State.PATROL_NAVIGATE)

    # def depth_image_callback(self, message):
    #     def convert_depth_image(ros_image):
    #         depth_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
    #         depth_array = np.array(depth_image, dtype=np.float32) * 0.001  # расстояние в метрах 0.001
    #         return depth_array
    #         # im = Image.fromarray(depth_array)
    #         # im = im.convert("L")
    #         # idx = str(i).zfill(4)
    #         # im.save(root+"/depth/frame{index}.png".format(index = idx))
    #         # i += 1
    #         # print("depth_idx: ", i)
    #     self.depth_images = self.depth_images[:min(len(self.depth_images), self.DEPTH_QUEUE_SIZE)]
    #     self.depth_images.append({'timestamp': {'secs': message.header.stamp.secs, 'nsecs': message.header.stamp.nsecs},
    #                               'image': convert_depth_image(message)})
    #
    # def target_callback(self, message):
    #     def draw_cross(img, x, y):
    #         CROSS_HALF = 2
    #         CROSS_HALF_LEN = 30
    #         for i in range(y - CROSS_HALF, y + CROSS_HALF + 1):
    #             for j in range(x - CROSS_HALF, x + CROSS_HALF + 1):
    #                 # Up
    #                 k = 0
    #                 while i - k >= 0 and k < CROSS_HALF_LEN:
    #                     img[i - k][j] = 1000
    #                     k += 1
    #                 # Down
    #                 k = 0
    #                 while i + k < 480 and k < CROSS_HALF_LEN:
    #                     img[i + k][j] = 1000
    #                     k += 1
    #                 # Left
    #                 k = 0
    #                 while j - k >= 0 and k < CROSS_HALF_LEN:
    #                     img[i][j - k] = 1000
    #                     k += 1
    #                 # Right
    #                 k = 0
    #                 while j + k < 640 and k < CROSS_HALF_LEN:
    #                     img[i][j + k] = 1000
    #                     k += 1
    #         return img
    #
    #     def get_min_range(img, x, y):
    #         SEARCH_RADIUS = 20
    #         x_start = max(x - SEARCH_RADIUS, 0)
    #         x_end = min(x + SEARCH_RADIUS, 639)
    #         y_start = max(y - SEARCH_RADIUS, 0)
    #         y_end = min(y + SEARCH_RADIUS, 479)
    #
    #         min_distance = 10
    #         for i in range(y_start, y_end + 1):
    #             for j in range(x_start, x_end + 1):
    #                 if img[i][j] > 0 and img[i][j] < min_distance:
    #                     min_distance = img[i][j]
    #         return min_distance
    #
    #
    #     # TODO: 1. Сделать поиск карты глубин, соответствующей данному timestamp-у +
    #     # TODO: 2. Сделать перевод координат цели на изображении в локальные координаты
    #     # TODO: 3. Сделать перевод координат цели на изображении в глобальные координаты
    #     message = message.data.split()
    #     secs = int(message[2])
    #     nsecs = int(message[3])
    #     i = 0
    #     while i < len(self.depth_images) and self.depth_images[i]['timestamp']['secs'] > secs:
    #         i += 1
    #     if i == len(self.depth_images):
    #         return
    #     start = i
    #     while i < len(self.depth_images) and self.depth_images[i]['timestamp']['secs'] == secs:
    #         i += 1
    #     end = i
    #     if start >= len(self.depth_images) or end >= len(self.depth_images):
    #         return
    #     min_i = start
    #     for i in range(start, end):
    #         if abs(self.depth_images[i]['timestamp']['nsecs'] - nsecs) < abs(self.depth_images[min_i]['timestamp']['nsecs'] - nsecs):
    #             min_i = i  # self.depth_images[i]['image'] - нужная карта глубины
    #     # self.depth_debug.publish(self.bridge.cv2_to_imgmsg(self.depth_images[i]['image']))
    #
    #     x_pix = int(message[0])
    #     y_pix = int(message[1])
    #     z_local = get_min_range(self.depth_images[min_i]['image'], x_pix, y_pix)
    #     self.depth_debug.publish(self.bridge.cv2_to_imgmsg(draw_cross(self.depth_images[min_i]['image'] * 100, x_pix, y_pix)))
    #
    #     w = 2 * math.sqrt(pow(z_local / math.cos(self.CAMERA_ANGLE_H / 2), 2) - pow(z_local, 2))
    #     h = 2 * math.sqrt(pow(z_local / math.cos(self.CAMERA_ANGLE_V / 2), 2) - pow(z_local, 2))
    #     x_local = (x_pix - 320) / 640 * w  # (w / 2)
    #     y_local = -1 * (y_pix - 240) / 480 * h  # (h / 2)
    #
    #     point = Point32()
    #     point.x, point.y, point.z = x_local, z_local, y_local
    #     cloud = PointCloud()
    #     cloud.header.stamp = rospy.Time.now()
    #     cloud.header.frame_id = "map"
    #     cloud.points.append(point)
    #     self.target_local_debug.publish(cloud)

    def target_callback(self, message):
        if math.isnan(message.x):
            if self.consecutive_detections > 0:
                self.consecutive_detections = 0
        else:
            self.consecutive_detections += 1
            target = np.array([message.x, message.y, message.z])
            if self.state == State.PURSUIT:
                self.pursuit_target = target
                self.state_timestamp = rospy.get_time()
            elif self.consecutive_detections >= self.PURSUIT_TRIGGER_COUNT:
                # self.state = State.PURSUIT
                self.set_state(State.PURSUIT)
                self.pursuit_target = target
            if self.state == State.SUSPICION:
                self.suspicion_target = target
                self.state_timestamp = rospy.get_time()
            elif self.state != State.PURSUIT and self.consecutive_detections >= self.SUSPICION_TRIGGER_COUNT:
                # self.state = State.SUSPICION
                self.set_state(State.SUSPICION)
                self.suspicion_target = target

    def target_callback_test(self, message):
        if message.data == '':
            self.state = 'patrol_navigate'
            self.pursuit_target = None
            self.patrol_target = None
            return
        message = message.data.split()
        self.pursuit_target = np.array(list(map(float, message)))
        self.state = 'pursuit'

    def on_shutdown_cb(self):
        rospy.logwarn("shutdown")
        self.land()
        rospy.loginfo("landing complete")


class State(Enum):
    PATROL_NAVIGATE = "PATROL_NAVIGATE"
    PATROL_SPIN = "PATROL_SPIN"
    SUSPICION = "SUSPICION"
    PURSUIT = "PURSUIT"
    SEARCH = "SEARCH"
    RTB = "RTB"


if __name__ == '__main__':
    controller = CopterController()
    try:
        controller.offboard_loop()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()

# TODO:
# 1. Сделать функцию преобразования координат цели на изображении в координаты относительно дрона, а затем в глобальные
# координаты
# 2. Сделать функцию определения момента для явного преследования цели
# 3. Сделать функцию проверки "подозреваемых" областей
