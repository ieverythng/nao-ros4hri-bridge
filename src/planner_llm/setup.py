from glob import glob

from setuptools import find_packages, setup


package_name = 'planner_llm'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juanbeck',
    maintainer_email='juanbeck@icloud.com',
    description='Planner LLM node for world-model-driven NAO planning',
    license='BSD-3-Clause',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'start_node = planner_llm.planner_node:main',
            'publish_fixture = planner_llm.fixture_publisher:main',
        ],
    },
)
