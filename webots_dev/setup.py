from setuptools import find_packages, setup
import os
from glob import glob

package_name = "webots_dev"

data_files = []
data_files.append(
    ("share/ament_index/resource_index/packages", ["resource/" + package_name])
)
data_files.append((os.path.join("share", package_name), ["package.xml"]))
data_files.append((os.path.join("share", package_name, "launch"), glob("launch/*.py")))
data_files.append((os.path.join("share", package_name, "worlds"), glob("worlds/*.wb*")))
data_files.append((os.path.join("share", package_name, "resource"), glob("resource/*")))
data_files.append(
    (os.path.join("share", package_name, "protos"), glob("protos/*.proto"))
)
data_files.append(
    (
        os.path.join("share", package_name, "protos", "meshes", "dae"),
        glob("protos/meshes/dae/*.dae"),
    )
)

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="deweykai",
    maintainer_email="deweykai5@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "add_noise_imu = webots_dev.add_noise_imu:main",
            "gps_odom = webots_dev.gps_odom:main",
            "gps_pose_publisher = webots_dev.gps_pose_publisher:main",
        ],
    },
)
