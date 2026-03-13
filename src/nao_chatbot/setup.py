from setuptools import find_packages, setup
from glob import glob

package_name = 'nao_chatbot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/ament_index/resource_index/pal_system_module',
            ['module/' + package_name]),
        ('share/' + package_name + '/module', ['module/' + package_name + '_module.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juanbeck',
    maintainer_email='juanbeck@icloud.com',
    description='Launch and operator-utility package for the NAO ROS4HRI migration stack',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
        'dev': [
            'pre-commit',
            'pytest',
            'ruff',
        ],
    },
    entry_points={
        'console_scripts': [
            'asr_push_to_talk_cli = nao_chatbot.asr_push_to_talk_cli:main',
            'robot_speech_debug = nao_chatbot.robot_speech_debug:main',
        ],
    },
)
