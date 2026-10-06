from setuptools import setup

package_name = 'lab1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chuang',
    maintainer_email='chuang@todo.todo',
    description='Introductory ROS 2 Humble node for MPHY0054 Lab 01.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello = lab1.hello:main',
        ],
    },
)
