from glob import glob
import os

from setuptools import setup

package_name = "simple_audio_capture"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="juanbeck",
    maintainer_email="juanbeck@icloud.com",
    description="Simple ROS 2 microphone capture node using GStreamer.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "audio_capture_node = simple_audio_capture.audio_capture_node:main",
        ],
    },
)
