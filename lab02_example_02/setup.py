import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'lab02_example_02'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ziting',
    maintainer_email='ziting@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lab02_example_02_pub = lab02_example_02.lab02_example_02_pub:main',
            'lab02_example_02_sub = lab02_example_02.lab02_example_02_sub:main',
        ],
    },
)
