from glob import glob
from setuptools import find_packages, setup

package_name = 'swarm_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Kavinaya',
    maintainer_email='kavinaya2409@gmail.com',
    description='Bringup and swarm commander between Unity and Crazyswarm2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'swarm_commander = swarm_control.swarm_commander:main',
        ],
    },
)
