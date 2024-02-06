from setuptools import setup, find_packages
import os
from glob import glob

package_name = "aruco"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="main",
    maintainer_email="jainab@oregonstate.edu",
    description="A package for a controller that scans tree branches by following up their main leader branch.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "aruco_goal_service = aruco.aruco_detection_srv:main",
        ],

    },
)
