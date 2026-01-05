from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'skyforge_controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dwidjaja',
    maintainer_email='dwidjaja@todo.todo',
    description='Skyforge Controllers',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'three_link_trajectory_publisher = skyforge_controllers.three_link_trajectory_publisher:main',
            'base_trajectory_publisher = skyforge_controllers.base_trajectory_publisher:main',
            'base_stepping_trajectory_publisher = skyforge_controllers.base_stepping_trajectory_publisher:main',


        ],
    },
)




# from setuptools import find_packages, setup

# package_name = 'skyforge_controllers'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         ('share/' + package_name + '/launch', [
#             'launch/controllers.launch.py']),
#         ('share/' + package_name + '/config', [
#             'config/three_link_arm_controllers.yaml']), 
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='dwidjaja',
#     maintainer_email='dwidjaja@todo.todo',
#     description='TODO: Package description',
#     license='TODO: License declaration',
#     tests_require=['pytest'],
#     entry_points={

#         'console_scripts': [
#             'three_link_trajectory_publisher = skyforge_controllers.three_link_trajectory_publisher:main',
#         ],
#     },
# )
