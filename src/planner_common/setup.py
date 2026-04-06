from setuptools import find_packages, setup


package_name = 'planner_common'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juanbeck',
    maintainer_email='juanbeck@icloud.com',
    description='Shared planner and world-model contract helpers',
    license='BSD-3-Clause',
    extras_require={
        'test': ['pytest'],
    },
)
