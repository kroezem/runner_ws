from setuptools import find_packages, setup

package_name = 'runner_battery'

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
    maintainer='matti',
    maintainer_email='matti@todo.todo',
    description='X1201 battery fuel gauge publisher',
    license='TODO',
    entry_points={
        'console_scripts': [
            'battery_node = runner_battery.battery_node:main',
        ],
    },
)
