import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ralphee_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('bringup/launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('bringup/config', '*config.[pxy][yma]*')))
    ],  
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
