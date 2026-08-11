from setuptools import setup

package_name = 'pnd_joint_state_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=['pnd_joint_state_demo'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 Publisher for robot states',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pnd_joint_state_publisher = pnd_joint_state_demo.robot_state_publisher_node:main',
        ],
    },
)
