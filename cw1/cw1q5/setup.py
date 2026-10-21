from setuptools import setup
import os
from glob import glob

package_name = 'cw1q5'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        # Include all rviz config files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kpach',
    maintainer_email='kpach@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cw1q5b_node = cw1q5.cw1q5b_node:main',
            'cw1q5d_node = cw1q5.cw1q5d_node:main',
        ],
    },
)