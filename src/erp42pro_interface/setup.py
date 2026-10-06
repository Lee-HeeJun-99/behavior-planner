from glob import glob
import os

from setuptools import setup


package_name = 'erp42pro_interface'

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
    maintainer='K-ROAD Team',
    maintainer_email='kroad@todo.todo',
    description='Safety-gated ERP42 Pro CAN bridge',
    license='Apache-2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'can_bridge_node = erp42pro_interface.can_bridge_node:main',
        ],
    },
)
