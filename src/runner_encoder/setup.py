from setuptools import find_packages, setup

package_name = 'runner_encoder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    # lgpio is provided externally by the apt package python3-lgpio.
    install_requires=['setuptools', 'rclpy', 'nav_msgs', 'std_msgs'],
    zip_safe=True,
    maintainer='matti',
    maintainer_email='matti@todo.todo',
    description='Wheel encoder odometry publisher for Runner',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'encoder_node = runner_encoder.encoder_node:main',
        ],
    },
)
