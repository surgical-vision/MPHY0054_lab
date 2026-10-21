from setuptools import setup, find_packages

package_name = 'cw1q4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='ROS 2 services for quaternion conversions.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'cw1q4_services = cw1q4.cw1q4_node:main',
        ],
    },
)