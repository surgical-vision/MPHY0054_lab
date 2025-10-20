from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'cw1q9'

setup(
    name=package_name,
    version='0.0.0',
    # CHANGE THIS SECTION
    # This now looks for packages in the main directory, not in a 'src' folder.
    packages=find_packages(exclude=['test']),
    # The package_dir line is no longer needed.
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tianchi',
    maintainer_email='mithrandir_chen@hotmail.com',
    description='The cw1q9 package updated for ROS 2',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'youbot_student_node = cw1q9.youbotKineStudent:main',
        ],
    },
)