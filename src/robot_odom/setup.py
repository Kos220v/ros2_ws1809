from setuptools import setup
import os
from glob import glob

package_name = 'robot_odom'

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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='admin',
    maintainer_email='admin@example.com',
    description='Одометрия: линейный путь из kolesa_control (/odom/vesc) + '
                'курс из кватерниона IMU -> /odom. Колёсный yaw не используется.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_node = robot_odom.odom_node:main',
        ],
    },
)
