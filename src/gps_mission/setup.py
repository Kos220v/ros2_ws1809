from setuptools import setup
import os
from glob import glob

package_name = 'gps_mission'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
]

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='admin',
    maintainer_email='admin@example.com',
    description='Миссия GPS-маршрута: waypoints.yaml -> Nav2 follow_gps_waypoints.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_mission_node = gps_mission.gps_mission_node:main',
        ],
    },
)
