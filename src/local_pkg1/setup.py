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
        ('share/' + package_name + '/launch', ['launch/local.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='K-ROAD (ERP42 Pro) localization: GPS + IMU + vehicle speed -> UTM position and heading',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 로컬라이제이션
            'tae_localization = local_pkg.tae_localization:main',
            # NGII VRS 용 GGA 중계 (/fix -> /gga -> ntrip_client)
            'gga_relay = local_pkg.gga_relay:main',
            # 맵 제작 도구
            'map = local_pkg.map_recorder:main',
            'path_smoothing = local_pkg.path_smoothing:main',
            'curvature_map = local_pkg.curve_ver2:main',
            # 확인용 그림 도구
            'gps_plot = local_pkg.gps_plot:main',
            'gps_heading_plot = local_pkg.heading_plot:main',
        ],
    },
)
