import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'bodenbot_scripts'


data_files = []
data_files.append(
    ('share/ament_index/resource_index/packages', ['resource/' + package_name])
)
data_files.append((os.path.join('share', package_name), ['package.xml']))
data_files.append((os.path.join('share', package_name, 'launch'), glob('launch/*.py')))
data_files.append((os.path.join('share', package_name, 'config'), glob('config/*')))
data_files.append(
    (os.path.join('share', package_name, 'config'), glob('config/*.json'))
)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='deweykai',
    maintainer_email='deweykai5@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'loader = bodenbot_scripts.loader:main',
            'mock_battery = bodenbot_scripts.mock_battery:main',
            'motor_driver = bodenbot_scripts.motor_driver:main',
            'docking_server = bodenbot_scripts.docking_server:main',
            'calibrate_striaght = bodenbot_scripts.calibrate_straight:main',
            'calibrate_box = bodenbot_scripts.calibrate_box:main',
            'gps_wpf = bodenbot_scripts.gps_wpf:main',
            'waypoint_recorder = bodenbot_scripts.waypoint_recorder:main',
            'demo_auto = bodenbot_scripts.demo_auto:main',
        ],
    },
)
