from setuptools import find_packages, setup

package_name = 'my_rover_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
	    ['launch/start_slam.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eugene',
    maintainer_email='eugene@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'scan_time_fixer = my_rover_slam.scan_time_fixer:main',
            'imu_odometry_node = my_rover_slam.imu_odometry_node:main',
            'static_tf_publisher  = my_rover_slam.static_tf_publisher:main',
            'wheel_odometry_node = my_rover_slam.wheel_odometry:main'

        ],
    },
)
