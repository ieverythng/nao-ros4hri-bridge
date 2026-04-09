from glob import glob

from setuptools import find_packages, setup


package_name = 'nao_world_model_enricher'


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
    description='Planner-facing world-model enrichment node',
    license='BSD-3-Clause',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'start_node = nao_world_model_enricher.world_model_enricher_node:main',
        ],
    },
)
