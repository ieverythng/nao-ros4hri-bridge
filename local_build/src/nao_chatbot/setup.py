from setuptools import find_packages, setup
from glob import glob

package_name = 'nao_chatbot'

setup(
    name=package_name,
    version='0.0.0',
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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'nao_rqt_bridge_node = nao_chatbot.nao_rqt_bridge:main',
            'mission_controller_node = nao_chatbot.mission_controller:main',
            'ollama_responder_node = nao_chatbot.ollama_responder:main',
            'ollama_node = nao_chatbot.ollama_node:main',
        ],
    },
)
