from setuptools import setup
import os
from glob import glob

package_name = 'img_processing'

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
    maintainer='rikic',
    maintainer_email='camille.ulrb@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_img_acquisition = img_processing.main_img:main',
            'node_img_processing = img_processing.main_img_processing:main'
        ],
    },
)
