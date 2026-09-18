#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
navigation.launch.py — навигационный слой: Nav2 + локализация (Nav2-стек
для движения по GPS-маршруту с объездом препятствий по лидару).

Запускает:
    ekf_odom            robot_localization EKF -> TF odom->base_link
                        (vx из /odom/vesc, ориентация из кватерниона /imu/data)
    ekf_map             robot_localization EKF -> TF map->odom (+ /odometry/gps)
    navsat_transform    /gps/fix -> /odometry/gps, сервисы /fromLL /toLL /datum
    controller_server   DWB, 10 Гц, /cmd_vel_nav -> /cmd_vel/auto (cmd_switcher)
    planner_server      NavFn, 1 Гц, окно костмапа 24×24 м по лидару
    behavior_server     BackUp + Wait (без Spin — гусеницы роют грунт)
    bt_navigator        дерево config/behavior_tree.xml
    waypoint_follower   action follow_gps_waypoints (Nav2 GPS waypoint follower)
    gps_mission         waypoints.yaml -> follow_gps_waypoints, статус, запись точек
    lifecycle_manager   автозапуск Nav2-серверов

НЕ запускается (экономия CPU на Raspberry Pi 5): smoother_server,
velocity_smoother, collision_monitor, map_server, amcl, slam_toolbox.

Аргументы:
    waypoints_file   маршрут (по умолчанию config/waypoints.yaml из project_start)
    loops            число ДОПОЛНИТЕЛЬНЫХ кругов (0 = один проход)
    cruise_speed     крейсерская скорость, м/с
    declination_deg  магнитное склонение, град (0, если STM32 уже учёл его)
    nav2_params      свой файл параметров Nav2
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    nav_share = get_package_share_directory('gps_navigator')
    mission_share = get_package_share_directory('gps_mission')

    declination_deg = float(
        LaunchConfiguration('declination_deg').perform(context))
    waypoints_file = LaunchConfiguration('waypoints_file').perform(context)
    if not waypoints_file:
        waypoints_file = os.path.join(
            get_package_share_directory('project_start'),
            'config', 'waypoints.yaml')

    ekf_params = os.path.join(nav_share, 'config', 'ekf_localization.yaml')
    nav2_params = LaunchConfiguration('nav2_params').perform(context) or \
        os.path.join(nav_share, 'config', 'nav2_params.yaml')
    bt_xml = os.path.join(nav_share, 'config', 'behavior_tree.xml')
    mission_params = os.path.join(mission_share, 'config', 'mission_params.yaml')

    # ------------------------------------------------- локализация (robot_localization)
    ekf_odom = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_odom',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[ekf_params],
        remappings=[('odometry/filtered', '/odometry/filtered')],
    )

    ekf_map = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_map',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[ekf_params],
    )

    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            ekf_params,
            {
                # переводим градусы (как в start.launch.py) в радианы
                'magnetic_declination_radians': math.radians(declination_deg),
            },
        ],
        remappings=[
            ('imu/data', '/imu/data'),
            # питаемся от отфильтрованных фиксов (gps_fix_gate): один
            # мультитрейновый прыжок не должен телепортировать map->odom
            ('gps/fix', '/gps/fix/filtered'),
            ('odometry/filtered', '/odometry/filtered'),
        ],
    )

    # ------------------------------------------------------------- Nav2
    # cmd_vel_nav -> /cmd_vel/auto: команды автопилота проходят через
    # cmd_switcher, где пульт ELRS имеет приоритет.
    nav2_remappings = [('cmd_vel', '/cmd_vel/auto')]
    nav2_common = dict(output='screen', parameters=[nav2_params])

    # ------------------------------------------------ GPS-гейт (антипрыжок)
    # Фильтрует /gps/fix: отбрасывает фиксы с плохой ковариацией (HDOP) и
    # «прыжки» (мультитрейн), которые телепортировали TF map->odom —
    # симптом: «Sensor origin ... out of map bounds ... cannot raytrace».
    gps_gate = Node(
        package='gps_navigator',
        executable='gps_fix_gate',
        name='gps_fix_gate',
        output='screen',
        parameters=[{'gps_topic': '/gps/fix',
                     'output_topic': '/gps/fix/filtered'}],
    )

    # ------------------------------------------------ курс по GPS (антидрейф)
    # Направление перемещения между фиксами -> /gps/heading (pose0 в ekf_map).
    # Не даёт дрейфу кватерниона IMU накапливаться в map, пока робот едет.
    gps_heading = Node(
        package='gps_navigator',
        executable='gps_heading',
        name='gps_heading',
        output='screen',
        parameters=[{'gps_topic': '/gps/fix/filtered',
                     'output_topic': '/gps/heading',
                     'frame_id': 'map'}],
    )

    controller_server = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', remappings=nav2_remappings, **nav2_common)

    planner_server = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', **nav2_common)

    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', **nav2_common)

    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator',
        remappings=nav2_remappings,
        parameters=[
            nav2_params,
            {'default_nav_through_poses_bt_xml': bt_xml,
             'default_nav_to_pose_bt_xml': bt_xml},
        ],
        output='screen')

    waypoint_follower = Node(
        package='nav2_waypoint_follower', executable='waypoint_follower',
        name='waypoint_follower', remappings=nav2_remappings, **nav2_common)

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[nav2_params],
    )

    # --------------------------------------------------------- миссия GPS
    gps_mission = Node(
        package='gps_mission',
        executable='gps_mission_node',
        name='gps_mission',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        parameters=[
            mission_params,
            {
                'waypoints_file': waypoints_file,
                'number_of_loops': int(
                    LaunchConfiguration('loops').perform(context)),
                'speed': float(
                    LaunchConfiguration('cruise_speed').perform(context)),
            },
        ],
    )

    return [
        ekf_odom,
        ekf_map,
        navsat,
        gps_gate,
        gps_heading,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
        gps_mission,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('waypoints_file', default_value='',
                              description='Путь к waypoints.yaml'),
        DeclareLaunchArgument('loops', default_value='0',
                              description='Число дополнительных кругов'),
        DeclareLaunchArgument('cruise_speed', default_value='0.6',
                              description='Крейсерская скорость, м/с'),
        DeclareLaunchArgument('declination_deg', default_value='0.0',
                              description='Магнитное склонение, град '
                                          '(0 — если учтено в STM32)'),
        DeclareLaunchArgument('nav2_params', default_value='',
                              description='Свой файл параметров Nav2'),
        OpaqueFunction(function=launch_setup),
    ])
