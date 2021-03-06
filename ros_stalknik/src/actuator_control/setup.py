from setuptools import setup
import os
from glob import glob

package_name = 'actuator_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'param'), glob('param/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nrodrigues64',
    maintainer_email='nico.r648@gmail.com',
    description='Publisher/subscriber for making the drone move to followw the car',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_actuator = actuator_control.main:main',
        ],
    },
)
