from setuptools import setup
import os
from glob import glob

package_name = 'gps_navigator'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'config'), glob('config/*')),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    (os.path.join('share', package_name, 'docs'), glob('docs/*')),
]

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='admin',
    maintainer_email='admin@example.com',
    description='Nav2 + robot_localization: движение по GPS-маршруту с объездом '
                'препятствий по лидару, предполётная проверка, документация.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'preflight_check = gps_navigator.preflight_check:main',
            'gps_heading = gps_navigator.gps_heading_node:main',
        ],
    },
)
