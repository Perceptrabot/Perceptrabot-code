from setuptools import find_packages, setup

package_name = 'turtlebot_serial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dimitrios',
    maintainer_email='mbouzoulasdimitris@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'teleop_node = turtlebot_serial.teleop_node:main',
        'arduino_bridge = turtlebot_serial.arduino_bridge_node:main',
        ],
    },
)
