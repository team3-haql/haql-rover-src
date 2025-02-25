from setuptools import find_packages, setup

package_name = 'px4_coms'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='default',
    maintainer_email='103225424+garthable@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'px4_advertiser = px4_coms.px4_advertiser:main',
            'px4_client = px4_coms.px4_client:main'
        ],
    },
)
