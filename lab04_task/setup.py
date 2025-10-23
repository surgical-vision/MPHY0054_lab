from setuptools import setup
# import necessary packages

package_name = 'lab04_task'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # install all launch files into the package’s shared folder so that ROS 2 can find
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chuang',
    maintainer_email='chuang@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # register your Python node function
        ],
    },
)