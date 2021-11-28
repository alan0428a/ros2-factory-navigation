from setuptools import setup
from glob import glob
import os

package_name = 'robot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.pgm'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alanyu',
    maintainer_email='pochun.alan.yu@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agv_demo=robot_navigation.agv_demo:main',
            'traffic_control=robot_navigation.traffic_control:main'
        ],
    },
)
