from setuptools import setup
import os
from glob import glob


package_name = 'lab04_solution'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # install all launch files into the package’s shared folder so that ROS 2 can find
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chuang',
    maintainer_email='chuang@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fkine_node = lab04_solution.fkine_node:main'
        ],
    },
)
