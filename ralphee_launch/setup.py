from setuptools import find_packages, setup
import glob
import os

package_name = 'ralphee_launch'

data_files = []
data_files.append(
    ('share/ament_index/resource_index/packages', ['resource/' + package_name])
)
data_files.append((os.path.join('share', package_name), ['package.xml']))
data_files.append((os.path.join('share', package_name, 'bringup/launch'), glob('bringup/launch/*.py')))
data_files.append((os.path.join('share', package_name, 'bringup/config'), glob('bringup/config/*')))
data_files.append(
    (os.path.join('share', package_name, 'bringup/config'), glob('bringup/config/*.json'))
)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='default',
    maintainer_email='daltonprokosch@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
