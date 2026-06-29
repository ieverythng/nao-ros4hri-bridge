from glob import glob
from pathlib import Path

from setuptools import find_packages, setup

package_name = 'nao_chatbot'
repo_root = Path(__file__).resolve().parents[2]
preloaded_environment_svg_files = [
    str(path)
    for path in sorted((repo_root / 'docs' / 'artifacts' / 'preloaded_environments').glob('*.svg'))
]

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        (
            'share/' + package_name + '/config/preloaded_environment_svgs',
            preloaded_environment_svg_files,
        ),
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
            'demo_rosout_filter = nao_chatbot.demo_rosout_filter:main',
            'preload_environment = nao_chatbot.preload_environment:main',
            'preloaded_environment_viewer = nao_chatbot.preloaded_environment_viewer:main',
            'robot_speech_debug = nao_chatbot.robot_speech_debug:main',
        ],
    },
)
