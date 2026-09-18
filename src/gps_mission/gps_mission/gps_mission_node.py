# -*- coding: utf-8 -*-

"""
gps_mission — тонкий клиент Nav2 для движения по GPS-маршруту.

Читает waypoints.yaml (lat/lon) и отправляет их в action-сервер
`follow_gps_waypoints` пакета nav2_waypoint_follower (nav2_msgs/
FollowGPSWaypoints). Сам waypoints_follower через сервис `/fromLL`
(robot_localization/navsat_transform) переводит lat/lon в систему `map`
и ведёт робота по точкам; объезд препятствий делает Nav2 по костмапам
из /scan.

Этот узел НЕ считает управление — только миссия: старт/стоп/пауза,
запись точек, статус. Потребление CPU — пренебрежимо мало.

Сервисы (std_srvs/Trigger):
    ~/start            — загрузить маршрут и отправить goal;
    ~/stop             — отменить goal;
    ~/record_waypoint  — дописать текущую GPS-точку в файл маршрута.

Топики:
    /gps/fix               (подписка, для записи точек)
    /gps_mission/status    (публикация состояния, ~2 Гц)
"""

import os

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geographic_msgs.msg import GeoPoseStamped
from nav2_msgs.action import FollowGPSWaypoints
from robot_localization.srv import SetDatum
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .waypoints_io import (
    append_waypoint,
    fill_local_coords,
    load_waypoints,
    route_length,
)


class GpsMission(Node):
    def __init__(self):
        super().__init__("gps_mission")

        p = self.declare_parameter
        p("waypoints_file", "")
        p("number_of_loops", 0)        # 0 = один проход; 1 = проехать 2 раза...
        p("start_index", 0)            # с какой точки начать
        p("speed", 0.6)                # м/с (передаётся в goal, инфо для Nav2)
        p("record_file", "~/gps_route.yaml")
        p("set_datum_from_first_waypoint", True)   # ноль `map` = 1-я точка
        p("datum_service", "/datum")   # сервис navsat_transform
        p("action_name", "follow_gps_waypoints")
        p("action_wait_timeout", 30.0)
        p("status_topic", "/gps_mission/status")
        p("gps_topic", "/gps/fix")

        g = self.get_parameter
        self.waypoints_file = self._expand(str(g("waypoints_file").value))
        self.number_of_loops = int(g("number_of_loops").value)
        self.start_index = int(g("start_index").value)
        self.speed = float(g("speed").value)
        self.record_file = self._expand(str(g("record_file").value))
        self.set_datum = bool(g("set_datum_from_first_waypoint").value)
        self.action_wait_timeout = float(g("action_wait_timeout").value)

        self.last_fix = None
        self.state = "IDLE"
        self.goal_handle = None
        self.current_waypoint = -1
        self.total_waypoints = 0
        self._last_status_t = 0.0

        self.create_subscription(NavSatFix, str(g("gps_topic").value),
                                 self._on_fix, 10)
        self.pub_status = self.create_publisher(
            String, str(g("status_topic").value), 10)

        self.client = ActionClient(self, FollowGPSWaypoints,
                                   str(g("action_name").value))
        self.datum_cli = self.create_client(SetDatum,
                                            str(g("datum_service").value))

        self.create_service(Trigger, "~/start", self._srv_start)
        self.create_service(Trigger, "~/stop", self._srv_stop)
        self.create_service(Trigger, "~/record_waypoint", self._srv_record)

        self.create_timer(0.5, self._publish_status)

        self.get_logger().info(
            "gps_mission готов. Маршрут: "
            f"{self.waypoints_file or '(не задан)'}; запуск: "
            "ros2 service call /gps_mission/start std_srvs/srv/Trigger")

    # ============================================================== сервисы
    def _srv_start(self, request, response):
        if self.goal_handle is not None and not self.goal_handle.done():
            response.success = False
            response.message = "Миссия уже выполняется"
            return response

        try:
            wps = load_waypoints(self.waypoints_file)
        except ValueError as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(response.message)
            return response

        if self.start_index >= len(wps):
            response.success = False
            response.message = (f"start_index={self.start_index} вне маршрута "
                                f"({len(wps)} точек)")
            return response

        # Ноль системы `map` — в первой точке маршрута: координаты в RViz
        # будут около (0; 0), проще понимать, где робот.
        if self.set_datum and self.datum_cli.wait_for_service(timeout_sec=5.0):
            req = SetDatum.Request()
            req.geo_pose.position.latitude = wps[0].lat
            req.geo_pose.position.longitude = wps[0].lon
            req.geo_pose.position.altitude = 0.0
            req.geo_pose.orientation.w = 1.0
            self.datum_cli.call_async(req)
            self.get_logger().info(
                f"datum установлен: {wps[0].lat:.7f}, {wps[0].lon:.7f}")
        elif self.set_datum:
            self.get_logger().warning(
                "Сервис /datum недоступен — ноль `map` будет по первому "
                "GPS-фиксу (navsat_transform)")

        # Оценка длины маршрута (ENU-аппроксимация вокруг первой точки) —
        # только для лога, Nav2 считает путь сам.
        fill_local_coords(wps, wps[0].lat, wps[0].lon)

        if not self.client.wait_for_server(
                timeout_sec=self.action_wait_timeout):
            response.success = False
            response.message = (
                "Action-сервер follow_gps_waypoints не отвечает — запущен ли "
                "nav2_waypoint_follower (navigation.launch.py)?")
            self.get_logger().error(response.message)
            return response

        goal = FollowGPSWaypoints.Goal()
        goal.number_of_loops = self.number_of_loops
        goal.goal_index = self.start_index
        for wp in wps:
            gp = GeoPoseStamped()
            gp.header.frame_id = "WGS84"
            gp.header.stamp = self.get_clock().now().to_msg()
            gp.pose.position.latitude = wp.lat
            gp.pose.position.longitude = wp.lon
            gp.pose.position.altitude = 0.0
            # Ориентация нулевая: у GPS-точки нет желаемого курса, поэтому
            # goal_checker в nav2_params.yaml настроен с свободным
            # yaw_goal_tolerance.
            gp.pose.orientation.w = 1.0
            goal.gps_poses.append(gp)

        self.total_waypoints = len(wps)
        self.state = "SENDING"
        send_future = self.client.send_goal_async(
            goal, feedback_callback=self._feedback)
        send_future.add_done_callback(self._goal_response)

        response.success = True
        response.message = (f"Маршрут отправлен в Nav2: {len(wps)} точек, "
                            f"~{route_length(wps):.0f} м, "
                            f"loops={self.number_of_loops}")
        self.get_logger().info(response.message)
        return response

    def _srv_stop(self, request, response):
        if self.goal_handle is not None and not self.goal_handle.done():
            self.goal_handle.cancel_goal_async()
            self.state = "CANCELLED"
            response.success = True
            response.message = "Отмена миссии отправлена"
        else:
            self.state = "IDLE"
            response.success = True
            response.message = "Активной миссии нет"
        return response

    def _srv_record(self, request, response):
        if self.last_fix is None:
            response.success = False
            response.message = "Нет GPS-фикса — точку записать нельзя"
            return response
        lat, lon = self.last_fix
        append_waypoint(self.record_file, lat, lon)
        response.success = True
        response.message = (f"Точка {lat:.7f},{lon:.7f} дописана в "
                            f"{self.record_file}")
        self.get_logger().info(response.message)
        return response

    # ============================================================== колбэки
    def _on_fix(self, msg: NavSatFix):
        if msg.status.status < 0:
            return
        self.last_fix = (msg.latitude, msg.longitude)

    def _goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.state = "REJECTED"
            self.get_logger().error("Nav2 отклонил goal маршрута")
            return
        self.goal_handle = handle
        self.state = "NAVIGATING"
        self.get_logger().info("Nav2 принял маршрут — начинаем движение")
        handle.get_result_async().add_done_callback(self._result)

    def _feedback(self, feedback_msg):
        idx = int(feedback_msg.feedback.current_waypoint)
        if idx != self.current_waypoint:
            self.current_waypoint = idx
            self.get_logger().info(
                f"Идём к точке {idx + 1}/{self.total_waypoints}")

    def _result(self, future):
        res = future.result().result
        self.goal_handle = None
        self.state = "DONE"
        if res.missed_waypoints:
            missed = ", ".join(str(m.index) for m in res.missed_waypoints)
            self.get_logger().warning(
                f"Маршрут завершён с пропущенными точками: {missed}")
        else:
            self.get_logger().info("МАРШРУТ ЗАВЕРШЁН: все точки пройдены")

    def _publish_status(self):
        msg = String()
        msg.data = (f"[{self.state}] точка "
                    f"{self.current_waypoint + 1}/{self.total_waypoints} "
                    f"gps={'есть' if self.last_fix else 'нет'}")
        self.pub_status.publish(msg)

    @staticmethod
    def _expand(path):
        return os.path.expanduser(path) if path else path


def main(args=None):
    rclpy.init(args=args)
    node = GpsMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
