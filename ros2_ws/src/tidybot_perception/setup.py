from setuptools import setup
import os
from glob import glob

package_name = 'tidybot_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team 7',
    maintainer_email='dooleymi@stanford.edu',
    description='Perception nodes for TidyBot2 - object detection, 3D localization, and grasp planning',
    license='MIT',
    entry_points={
        'console_scripts': [
            'detector_node = tidybot_perception.detector_node:main',
            'object_localizer_node = tidybot_perception.object_localizer_node:main',
            'grasp_planner_node = tidybot_perception.grasp_planner_node:main',
        ],
    },
)
