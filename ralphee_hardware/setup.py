from setuptools import find_packages, setup

package_name = 'ralphee_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'moteus', 'pyserial', "django>2.0; os_name == 'nt'", "usb"],
    zip_safe=True,
    maintainer='default',
    maintainer_email='daltonprokosch@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hardware_controller = ralphee_hardware.ralphee_hardware_controller:main'
        ],
    },
)
