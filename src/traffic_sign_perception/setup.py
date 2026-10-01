from glob import glob
import os

from setuptools import setup

package_name = 'traffic_sign_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'weights'), glob('weights/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='K-ROAD Team',
    maintainer_email='kroad@todo.todo',
    description='YOLO speed-limit sign perception node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'speed_sign_node = traffic_sign_perception.speed_sign_node:main',
        ],
    },
)
