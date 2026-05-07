from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'tbot3_nav_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anilegin',
    maintainer_email='anilegin@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'metrics_collector = tbot3_nav_monitor.metrics_collector:main',
            'csv_logger = tbot3_nav_monitor.csv_logger:main',
            'adaptive_controller = tbot3_nav_monitor.adaptive_controller:main',
            'metrics_visualizer = tbot3_nav_monitor.metrics_visualizer:main',
            'batch_goal_sender = tbot3_nav_monitor.batch_goal_sender:main',
            'ml_predictor = tbot3_nav_monitor.ml_predictor:main',
            'web_dashboard = tbot3_nav_monitor.web_dashboard:main',
        ],
    },
)
