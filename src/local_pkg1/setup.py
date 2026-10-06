from setuptools import setup

package_name = 'local_pkg1'

setup(
    name=package_name,
    version='0.0.0',
    packages=['local_pkg'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    test_suite='test',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                  'tae_localization = local_pkg.tae_localization:main',
                  'heading_estimator = local_pkg.heading_estimator:main',
                  'position_estimator = local_pkg.position_estimator:main',
        ],
    },
)
