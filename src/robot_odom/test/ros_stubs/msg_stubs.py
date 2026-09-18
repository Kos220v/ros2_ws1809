# -*- coding: utf-8 -*-

"""
Мини-заглушки rclpy и ROS-сообщений для тестов БЕЗ установленной ROS 2.

Заглушки повторяют только тот интерфейс, который реально использует
robot_odom/odom_node.py. Это позволяет исполнить НАСТОЯЩИЙ код узла
(колбэки, интегрирование, watchdog, публикацию /odom) на машине без ROS —
например, в CI или в этом репозитории. На роботе тесты идут через
`colcon test`, где доступен настоящий rclpy.

Список сообщений/полей сверен с odom_node.py:
    sensor_msgs/Imu  (orientation, angular_velocity)
    nav_msgs/Odometry (header, child_frame_id, pose.*, twist.*, covariance)
    geometry_msgs/TwistWithCovariance, TransformStamped
    tf2_msgs/TFMessage
"""


class _Vector3:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _Quaternion:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0


class _Point:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _Pose:
    def __init__(self):
        self.position = _Point()
        self.orientation = _Quaternion()


class _PoseWithCovariance:
    def __init__(self):
        self.pose = _Pose()
        self.covariance = [0.0] * 36


class _Twist:
    def __init__(self):
        self.linear = _Vector3()
        self.angular = _Vector3()


class _TwistWithCovariance:
    def __init__(self):
        self.twist = _Twist()
        self.covariance = [0.0] * 36


class _Header:
    def __init__(self):
        self.stamp = None
        self.frame_id = ""


class Imu:
    def __init__(self):
        self.header = _Header()
        self.orientation = _Quaternion()
        self.angular_velocity = _Vector3()
        self.linear_acceleration = _Vector3()


class Odometry:
    def __init__(self):
        self.header = _Header()
        self.child_frame_id = ""
        self.pose = _PoseWithCovariance()
        self.twist = _TwistWithCovariance()


class TwistWithCovariance(_TwistWithCovariance):
    pass


class TransformStamped:
    def __init__(self):
        self.header = _Header()
        self.child_frame_id = ""
        self.transform = type("T", (), {})()
        self.transform.translation = _Vector3()
        self.transform.rotation = _Quaternion()


class TFMessage:
    def __init__(self):
        self.transforms = []
