#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
route.launch.py — ПОЛНЫЙ запуск робота для движения по GPS-маршруту.

Собирает два слоя:
    project_start/start.launch.py      железо: ELRS, VESC (kolesa_control),
                                       IMU, GNSS, лидар, robot_state_publisher,
                                       cmd_switcher, relay_reliable, robot_odom
    gps_navigator/navigation.launch.py локализация (2×EKF + navsat_transform)
                                       и Nav2 (контроллер, планер, поведения,
                                       BT, waypoint_follower) + gps_mission

Запуск на роботе:
    ros2 launch gps_navigator route.launch.py
    ros2 launch gps_navigator route.launch.py waypoints_file:=/home/pi/route.yaml loops:=1

Старт миссии (после предполётной проверки!):
    ros2 service call /gps_mission/start std_srvs/srv/Trigger

Аргументы (основные):
    waypoints_file    маршрут waypoints.yaml (умолч. — из project_start)
    loops             число ДОПОЛНИТЕЛЬНЫХ кругов (0 = один проход)
    cruise_speed      крейсерская скорость, м/с
    declination_deg   магнитное склонение, град (0 — если учтено в STM32)
    use_gps           false — стенд без GNSS (маршрут не поедет)
    lidar_delay       задержка старта лидара, с (IMU должна откалиброваться)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    start_launch = os.path.join(
        get_package_share_directory('project_start'), 'launch', 'start.launch.py')
    nav_launch = os.path.join(
        get_package_share_directory('gps_navigator'), 'launch',
        'navigation.launch.py')

    declination = LaunchConfiguration('declination_deg')

    return LaunchDescription([
        DeclareLaunchArgument('waypoints_file', default_value='',
                              description='Путь к waypoints.yaml'),
        DeclareLaunchArgument('loops', default_value='0',
                              description='Число дополнительных кругов'),
        DeclareLaunchArgument('cruise_speed', default_value='0.6'),
        DeclareLaunchArgument('declination_deg', default_value='0.0',
                              description='Магнитное склонение, град'),
        DeclareLaunchArgument('use_gps', default_value='true'),
        DeclareLaunchArgument('lidar_delay', default_value='10.0'),
        DeclareLaunchArgument('gps_port', default_value='/dev/ttyAMA2'),
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyAMA1'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('imu_yaw_offset_deg', default_value='-48.0'),
        DeclareLaunchArgument('nav2_params', default_value=''),

        # ------------------------------------------------------ слой железа
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(start_launch),
            launch_arguments={
                'use_gps': LaunchConfiguration('use_gps'),
                'lidar_delay': LaunchConfiguration('lidar_delay'),
                'gps_port': LaunchConfiguration('gps_port'),
                'imu_port': LaunchConfiguration('imu_port'),
                'lidar_port': LaunchConfiguration('lidar_port'),
                'declination_deg': declination,
                'imu_yaw_offset_deg': LaunchConfiguration('imu_yaw_offset_deg'),
                # TF odom->base_link публикует ekf_odom (robot_localization),
                # поэтому robot_odom свой TF НЕ издаёт и не конфликтует.
                'odom_publish_tf': 'false',
                'odom_yaw_mode': 'absolute',
            }.items(),
        ),

        # ------------------------------------------------- навигация (Nav2)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch),
            launch_arguments={
                'waypoints_file': LaunchConfiguration('waypoints_file'),
                'loops': LaunchConfiguration('loops'),
                'cruise_speed': LaunchConfiguration('cruise_speed'),
                'declination_deg': declination,
                'nav2_params': LaunchConfiguration('nav2_params'),
            }.items(),
        ),
    ])
