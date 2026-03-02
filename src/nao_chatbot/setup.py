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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juanbeck',
    maintainer_email='juanbeck@icloud.com',
    description='ROS 2 chatbot orchestration and bridges for NAO + ROS4HRI',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
        'asr': [
            'sounddevice',
            'vosk',
        ],
        'dev': [
            'pre-commit',
            'pytest',
            'ruff',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_controller_node = nao_chatbot.mission_controller:main',
            'ollama_chatbot_node = nao_chatbot.ollama_chatbot:main',
            'asr_vosk_node = nao_chatbot.asr_vosk:main',
        ],
    },
)
