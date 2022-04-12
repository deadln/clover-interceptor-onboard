#!/usr/bin/env python
import rospy
from clover import srv
from std_srvs.srv import Trigger
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge

import numpy as np
import math
import random


node_name = "copter_node"

class CopterController():
    def __init__(self):
        rospy.init_node(node_name)
        rospy.loginfo(node_name + " started")

        self.__get_telemetry__ = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        # self.__navigate__ = rospy.ServiceProxy('navigate', srv.Navigate)
        # self.__set_position__ = rospy.ServiceProxy('set_position', srv.SetPosition)
        # self.__set_velocity__ = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
        # self.__land__ = rospy.ServiceProxy('land', Trigger)

        self.bridge = CvBridge()

        self.FREQUENCY = 10
        self.DEPTH_QUEUE_SIZE = 500
        self.CAMERA_ANGLE_H = 1.5009831567151235
        self.CAMERA_ANGLE_V = 0.9948376736367679


        self.X_NORM = np.array([1, 0, 0])
        self.SPIN_TIME = 8
        self.SPIN_RATE = math.pi / self.SPIN_TIME
        self.PATROL_SPEED = 0.3
        self.INTERCEPTION_SPEED = 0.5

        # TODO: парсить данные о полётной зоне из txt или launch файла
        self.low_left_corner = np.array([0.0, 0.0, 0.8])
        self.up_right_corner = np.array([3.0, 6.0, 4.2])
        # self.min_height = 0.8
        # self.max_height = 4.2
        self.state = ""
        self.patrol_target = None
        self.spin_start = None
        self.pursuit_target = None
        self.pursuit_target_detections = []
        self.depth_images = []

        self.depth_debug = rospy.Publisher("debug/depth", Image, queue_size=10)
        self.target_local_debug = rospy.Publisher("debug/target_position_local", PointCloud, queue_size=10)
        self.target_global_debug = rospy.Publisher("debug/target_position_global", PointCloud, queue_size=10)
        rospy.Subscriber('drone_detection/target', String, self.target_callback)
        # rospy.Subscriber('drone_detection/false_target', String, self.target_callback_test)
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_image_callback)

        rospy.on_shutdown(self.on_shutdown_cb)

    def get_telemetry(self, frame_id='aruco_map'):
        telemetry = self.__get_telemetry__(frame_id=frame_id)
        return telemetry

    def get_position(self, frame_id='aruco_map'):
        telemetry = self.__get_telemetry__(frame_id=frame_id)
        return np.array([telemetry.x, telemetry.y, telemetry.z])

    # def navigate(self, target=np.array([0, 0, 2]), speed=0.5, yaw=float('nan'), yaw_rate=0.0, auto_arm=False, frame_id='aruco_map'):
    #     self.__navigate__(x=target[0], y=target[1], z=target[2], speed=speed, yaw=yaw, yaw_rate=yaw_rate, auto_arm=auto_arm, frame_id=frame_id)
    #
    # def set_position(self, target=np.array([0, 0, 2]), speed=0.5, yaw=float('nan'), yaw_rate=0, auto_arm=False, frame_id='aruco_map'):
    #     self.__set_position__(x=target[0], y=target[1], z=target[2], speed=speed, yaw=yaw, yaw_rate=yaw_rate, auto_arm=auto_arm, frame_id=frame_id)
    #
    # def set_velocity(self, target=np.array([0, 0, 0]), yaw=float('nan'), yaw_rate=0, auto_arm=False, frame_id='aruco_map'):
    #     self.__set_velocity__(vx=target[0], vy=target[1], vz=target[2], yaw=yaw, yaw_rate=yaw_rate, auto_arm=auto_arm, frame_id=frame_id)
    #
    # def land(self):
    #     return self.__land__()

    def offboard_loop(self):
        self.takeoff()

        rate = rospy.Rate(self.FREQUENCY)
        while True:  # not rospy.is_shutdown():
            if not self.is_inside_patrol_zone():
                self.return_to_patrol_zone()
                continue
            if self.state == "patrol_navigate":  # Полёт к точке патрулирования
                if self.patrol_target is None:
                    self.set_patrol_target()
                    rospy.loginfo(f"New patrol target {self.patrol_target}")
                    # self.navigate(self.patrol_target[0], self.patrol_target[1], self.patrol_target[2], self.get_yaw_angle(self.X_NORM, self.patrol_target))
                    # self.navigate(self.patrol_target, yaw=float('nan'), yaw_rate=self.SPIN_RATE)
                elif self.is_navigate_target_reached():
                    rospy.loginfo("Patrol target reached")
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
                error = self.pursuit_target + np.array([0 ,0 ,1]) - position
                velocity = error / np.linalg.norm(error) * self.INTERCEPTION_SPEED
                rospy.loginfo(f"In puruit. Interception velocity {velocity}")
                # self.set_velocity(velocity, yaw=self.get_yaw_angle(self.X_NORM, self.pursuit_target))

            if self.state == "suspicion":  # Проверка места, в котором с т.з. нейросети "мелькнул дрон"
                pass
            if self.state == "search":  # Поиск утерянной цели
                pass

            if self.state == "rtb":  # Возвращение на базу
                pass

            # rate.sleep()

    def takeoff(self):
        # self.navigate(frame_id="", auto_arm = True)
        # rospy.sleep(5)
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
        velocity += list(map(int, position < self.low_left_corner))
        velocity += list(map(int, position > self.up_right_corner))
        velocity *= self.INTERCEPTION_SPEED
        # self.set_velocity(velocity)
        rospy.logwarn(f"OUT OF PATROL ZONE. RETURN VECTOR {velocity}")

    def is_navigate_target_reached(self, tolerance):
        position = self.get_position(frame_id='navigate_target')
        return np.linalg.norm(position) < tolerance
        # return math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance

    def is_inside_patrol_zone(self):
        position = self.get_position()
        return all(position >= self.low_left_corner) and all(position <= self.up_right_corner)


    def depth_image_callback(self, message):
        def convert_depth_image(ros_image):
            depth_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
            depth_array = np.array(depth_image, dtype=np.float32) * 0.001  # расстояние в метрах 0.001
            return depth_array
            # im = Image.fromarray(depth_array)
            # im = im.convert("L")
            # idx = str(i).zfill(4)
            # im.save(root+"/depth/frame{index}.png".format(index = idx))
            # i += 1
            # print("depth_idx: ", i)
        self.depth_images = self.depth_images[:min(len(self.depth_images), self.DEPTH_QUEUE_SIZE)]
        self.depth_images.append({'timestamp': {'secs': message.header.stamp.secs, 'nsecs': message.header.stamp.nsecs},
                                  'image': convert_depth_image(message)})

    def target_callback(self, message):
        def draw_cross(img, x, y):
            CROSS_HALF = 2
            CROSS_HALF_LEN = 30
            for i in range(y - CROSS_HALF, y + CROSS_HALF + 1):
                for j in range(x - CROSS_HALF, x + CROSS_HALF + 1):
                    # Up
                    k = 0
                    while i - k >= 0 and k < CROSS_HALF_LEN:
                        img[i - k][j] = 1000
                        k += 1
                    # Down
                    k = 0
                    while i + k < 480 and k < CROSS_HALF_LEN:
                        img[i + k][j] = 1000
                        k += 1
                    # Left
                    k = 0
                    while j - k >= 0 and k < CROSS_HALF_LEN:
                        img[i][j - k] = 1000
                        k += 1
                    # Right
                    k = 0
                    while j + k < 640 and k < CROSS_HALF_LEN:
                        img[i][j + k] = 1000
                        k += 1
            return img

        def get_min_range(img, x, y):
            SEARCH_RADIUS = 20
            x_start = max(x - SEARCH_RADIUS, 0)
            x_end = min(x + SEARCH_RADIUS, 639)
            y_start = max(y - SEARCH_RADIUS, 0)
            y_end = min(y + SEARCH_RADIUS, 479)

            min_distance = 10
            for i in range(y_start, y_end + 1):
                for j in range(x_start, x_end + 1):
                    if img[i][j] > 0 and img[i][j] < min_distance:
                        min_distance = img[i][j]
            return min_distance

        def turn_vector(vect, axis, angle):
            axis = axis / np.linalg.norm(axis)
            rot_matrix = np.array([
                [math.cos(angle) + (1 - math.cos(angle)) * axis[0] ** 2,
                 (1 - math.cos(angle)) * axis[0] * axis[1] - math.sin(angle) * axis[2],
                 (1 - math.cos(angle)) * axis[0] * axis[2] + math.sin(angle) * axis[1]],
                [(1 - math.cos(angle)) * axis[1] * axis[0] + math.sin(angle) * axis[2],
                 math.cos(angle) + (1 - math.cos(angle)) * axis[1] ** 2,
                 (1 - math.cos(angle)) * axis[1] * axis[2] - math.sin(angle) * axis[0]],
                [(1 - math.cos(angle)) * axis[2] * axis[0] - math.sin(angle) * axis[1],
                 (1 - math.cos(angle)) * axis[2] * axis[1] + math.sin(angle) * axis[0],
                 math.cos(angle) + (1 - math.cos(angle)) * axis[2] ** 2]
            ])
            return np.dot(vect, rot_matrix)

        # TODO: 1. Сделать поиск карты глубин, соответствующей данному timestamp-у +
        # TODO: 2. Сделать перевод координат цели на изображении в локальные координаты +
        # TODO: 3. Сделать перевод координат цели на изображении в глобальные координаты
        message = message.data.split()
        secs = int(message[2])
        nsecs = int(message[3])
        i = 0
        while i < len(self.depth_images) and self.depth_images[i]['timestamp']['secs'] > secs:
            i += 1
        if i == len(self.depth_images):
            return
        start = i
        while i < len(self.depth_images) and self.depth_images[i]['timestamp']['secs'] == secs:
            i += 1
        end = i
        if start >= len(self.depth_images) or end >= len(self.depth_images):
            return
        min_i = start
        for i in range(start, end):
            if abs(self.depth_images[i]['timestamp']['nsecs'] - nsecs) < abs(self.depth_images[min_i]['timestamp']['nsecs'] - nsecs):
                min_i = i  # self.depth_images[i]['image'] - нужная карта глубины
        # self.depth_debug.publish(self.bridge.cv2_to_imgmsg(self.depth_images[i]['image']))

        x_pix = int(message[0])
        y_pix = int(message[1])
        z_local = get_min_range(self.depth_images[min_i]['image'], x_pix, y_pix)
        self.depth_debug.publish(self.bridge.cv2_to_imgmsg(draw_cross(self.depth_images[min_i]['image'] * 100, x_pix, y_pix)))

        w = 2 * math.sqrt(pow(z_local / math.cos(self.CAMERA_ANGLE_H / 2), 2) - pow(z_local, 2))
        h = 2 * math.sqrt(pow(z_local / math.cos(self.CAMERA_ANGLE_V / 2), 2) - pow(z_local, 2))
        x_local = (x_pix - 320) / 640 * w  # (w / 2)
        y_local = -1 * (y_pix - 240) / 480 * h  # (h / 2)

        point = Point32()
        point.x, point.y, point.z = x_local, z_local, y_local
        cloud = PointCloud()
        cloud.header.stamp = rospy.Time.now()
        cloud.header.frame_id = "map"
        cloud.points.append(point)
        self.target_local_debug.publish(cloud)

        # Перевод в координаты в пространстве
        telemetry = self.get_telemetry()
        x_norm = np.array([1, 0, 0])
        y_norm = np.array([0, 1, 0])
        z_norm = np.array([0, 0, 1])

        turn_1 = turn_vector(x_norm, -z_norm, telemetry.yaw)
        axis = turn_vector(y_norm, -z_norm, telemetry.yaw)
        turn_2 = turn_vector(turn_1, -axis, telemetry.pitch)
        z_global_vector = turn_2.copy()
        x_global_vector = turn_vector(-axis, -z_global_vector, telemetry.roll)
        y_global_vector = np.cross(x_global_vector, z_global_vector)

        target_position = x_global_vector * x_local + y_global_vector * y_local + z_global_vector * z_local
        target_position += self.get_position()
        point = Point32()
        point.x, point.y, point.z = target_position[0], target_position[1], target_position[2]
        cloud = PointCloud()
        cloud.header.stamp = rospy.Time.now()
        cloud.header.frame_id = "aruco_map"
        cloud.points.append(point)
        self.target_global_debug.publish(cloud)





    def target_callback_test(self, message):
        message = message.split()
        self.pursuit_target = np.array(list(map(float, message[:2])))

    def on_shutdown_cb(self):
        rospy.logwarn("shutdown")
        # self.land()
        rospy.loginfo("landing complete")


if __name__ == '__main__':
    controller = CopterController()
    try:
        CopterController.offboard_loop()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()

# TODO:
# 1. Сделать функцию преобразования координат цели на изображении в координаты относительно дрона, а затем в глобальные
# координаты
# 2. Сделать функцию определения момента для явного преследования цели
# 3. Сделать функцию проверки "подозреваемых" областей