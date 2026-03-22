#!/usr/bin/env python3

from setuptools import find_packages, setup


NAME = "kb_skills"


setup(
    name=NAME,
    version="0.1.0",
    license="Apache-2.0",
    description="KnowledgeCore client helpers and formal skill metadata",
    author="juanbeck",
    author_email="juanbeck@icloud.com",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/" + NAME, ["package.xml"]),
        ("share/ament_index/resource_index/packages", ["res/" + NAME]),
    ],
    tests_require=["pytest"],
    install_requires=["setuptools"],
    zip_safe=True,
)
