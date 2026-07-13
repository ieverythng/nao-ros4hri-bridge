from setuptools import find_packages, setup

package_name = 'ab_harness'

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
    description='Pure-Python AB-grounded agent harness contracts and gates',
    license='BSD-3-Clause',
    extras_require={'test': ['pytest']},
)
