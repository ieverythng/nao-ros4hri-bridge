from setuptools import setup

package_name = 'asr_vosk'


setup(
    name=package_name,
    version='2.4.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/00-defaults.yml']),
        ('share/' + package_name + '/launch', [
            'launch/asr_vosk.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Séverin Lemaignan',
    maintainer_email='severin.lemaignan@pal-robotics.com',
    description='The asr_vosk package',
    license='Apache License 2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'asr_vosk = asr_vosk.node_vosk:main',
        ],
    },
)
