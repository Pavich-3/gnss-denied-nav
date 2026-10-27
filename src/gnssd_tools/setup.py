from setuptools import setup

package_name = 'gnssd_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Pavlo Baranchuk',
    maintainer_email='pavlobaranchuk03072004@gmail.com',
    description='Project nodes and evaluation scripts',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'odom_noise_node = gnssd_tools.odom_noise_node:main',
            'route_driver_node = gnssd_tools.route_driver_node:main',
            'run_info_writer_node = gnssd_tools.run_info_writer_node:main',
            'imu_covariance_node = gnssd_tools.imu_covariance_node:main',
            'loop_closure_monitor_node = gnssd_tools.loop_closure_monitor_node:main',
            'resource_logger_node = gnssd_tools.resource_logger_node:main',
            'sensor_monitor_node = gnssd_tools.sensor_monitor_node:main',
            'scan_gate_node = gnssd_tools.scan_gate_node:main',
            'node_census_node = gnssd_tools.node_census_node:main',
            'map_manager_node = gnssd_tools.map_manager_node:main',
            'scripted_operator_node = gnssd_tools.scripted_operator_node:main',
            'nav2_goal_sender_node = gnssd_tools.nav2_goal_sender_node:main',
        ],
    },
)
