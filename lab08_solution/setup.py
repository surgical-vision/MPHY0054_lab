import os
from glob import glob

from setuptools import setup, find_packages

package_name = 'lab08_solution'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chuang',
    maintainer_email='chuang@todo.todo',
    description='Jacobian-transpose IK demo for the youBot arm (ROS 2 Foxy).',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lab08_solution = lab08_solution.lab08_solution:main',
        ],
    },
)
