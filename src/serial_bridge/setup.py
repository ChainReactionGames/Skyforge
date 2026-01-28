from setuptools import setup, find_packages

package_name = 'serial_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/serial_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dwidjaja',
    maintainer_email='denzel.ezekiel@gmail.com',
    description='Serial bridge for ESP32 LQR',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bridge_node = serial_bridge.bridge_node:main',
        ],
    },
)
