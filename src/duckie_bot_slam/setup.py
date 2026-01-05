from setuptools import find_packages, setup

package_name = 'duckie_bot_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vaibhav',
    maintainer_email='vaibhav.kum5789@gmail.com',
    description='EKF SLAM and navigation for Duckie Bot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ekf_slam_node = duckie_bot_slam.ekf_slam.ekf_slam_ros:main',
            'landmark_sensor_node = duckie_bot_slam.ekf_slam.landmark_node:main',
            'waypoint_controller_node = duckie_bot_slam.ekf_slam.waypoint:main',
        ],
    },
)
