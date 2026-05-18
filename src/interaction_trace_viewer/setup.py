from glob import glob

from setuptools import find_packages, setup


package_name = 'interaction_trace_viewer'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juanbeck',
    maintainer_email='juanbeck@icloud.com',
    description='Simple interaction trace viewer with JSONL and HTML export',
    license='BSD-3-Clause',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'trace_node = interaction_trace_viewer.trace_node:main',
            'render_html = interaction_trace_viewer.render_html:main',
        ],
    },
)
