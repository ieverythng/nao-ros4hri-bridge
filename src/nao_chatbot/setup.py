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
            'nao_rqt_bridge_node = nao_chatbot.nao_rqt_bridge:main',
            'dialogue_manager_node = nao_chatbot.dialogue_manager:main',
            'mission_controller_node = nao_chatbot.mission_controller:main',
            'chat_skill_server_node = nao_chatbot.chat_skill_server:main',
            'ollama_responder_node = nao_chatbot.ollama_responder:main',
            'ollama_node = nao_chatbot.ollama_node:main',
            'asr_vosk_node = nao_chatbot.asr_vosk:main',
            'laptop_asr_node = nao_chatbot.asr_vosk:main',
        ],
    },
)
