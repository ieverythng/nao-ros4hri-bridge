#!/usr/bin/env python3

from glob import glob

from setuptools import find_packages, setup


NAME = "nao_look_at"


setup(
    name=NAME,
    version="0.1.0",
    license="Apache-2.0",
    description="NAO-specific scaffold for the ROS4HRI look_at skill",
    author="juanbeck",
    author_email="juanbeck@icloud.com",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/" + NAME, ["package.xml"]),
        ("share/ament_index/resource_index/packages", ["res/" + NAME]),
        ("share/ament_index/resource_index/pal_system_module", ["module/" + NAME]),
        ("share/" + NAME + "/launch", glob("launch/*.launch.py")),
        ("share/" + NAME + "/module", ["module/" + NAME + "_module.yaml"]),
        (
            "share/ament_index/resource_index/pal_configuration." + NAME,
            ["config/" + NAME],
        ),
        ("share/" + NAME + "/config", ["config/00-defaults.yml"]),
    ],
    tests_require=["pytest"],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "start_skill = nao_look_at.start_skill:main",
        ],
    },
)
