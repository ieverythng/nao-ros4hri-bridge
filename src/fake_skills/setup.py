from glob import glob

from setuptools import find_packages, setup


NAME = 'fake_skills'


setup(
    name=NAME,
    version='0.1.0',
    description='Deterministic fake skill servers and engine for planner validation',
    license='Apache-2.0',
    maintainer='juanbeck',
    maintainer_email='juanbeck@icloud.com',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + NAME]),
        ('share/' + NAME, ['package.xml']),
        ('share/' + NAME + '/config', glob('config/*.yaml')),
        ('share/' + NAME + '/launch', glob('launch/*.launch.py')),
    ],
    entry_points={
        'console_scripts': [
            'run_fake_skill_server = fake_skills.run_fake_skill_server:main',
        ],
    },
)
