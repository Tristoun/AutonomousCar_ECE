from setuptools import find_packages, setup

package_name = 'naive_trajectory'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
	    ['launch/traj.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tristan',
    maintainer_email='tristan@tristanjean.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'naive_traj_lidar = naive_trajectory.naive_traj_lidar:main', 
            'pure_pursuit = naive_trajectory.pure_pursuit:main',
            'planner = naive_trajectory.planner:main',
        ],
    },
)
